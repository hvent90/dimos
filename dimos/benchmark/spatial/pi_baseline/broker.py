# Copyright 2026 Dimensional Inc.
"""Case-bound tool broker for the constrained baseline container."""

from __future__ import annotations

import base64
from collections.abc import Callable
from dataclasses import dataclass
from io import BytesIO
import os
from pathlib import Path
import re
import stat
import struct
import warnings
import zlib

from PIL import Image

from dimos.benchmark.spatial.pi_baseline.config import PromptMode
from dimos.benchmark.spatial.pi_baseline.podman import PersistentPodmanCase
from dimos.benchmark.spatial.pi_baseline.transaction import AnswerReceipt, AnswerTransaction

VISUALIZATION_REQUIRED_ERROR = "visualization_required_before_submission"
VISUALIZATION_FORBIDDEN_ERROR = "visualization_forbidden"


class PolicyViolationError(ValueError):
    """A stable, non-oracle tool/policy failure visible to the adapter."""

    def __init__(self, code: str) -> None:
        super().__init__(code)
        self.code = code


@dataclass(frozen=True)
class BrokerLimits:
    max_command_bytes: int = 4_096
    max_output_bytes: int = 16_384
    # Leave room for base64 and the NDJSON envelope in the controller's 64 KiB frame.
    max_image_bytes: int = 45 * 1024
    max_image_width: int = 4_096
    max_image_height: int = 4_096
    max_image_pixels: int = 8_388_608
    max_images: int = 8


class CaseBroker:
    """Expose exactly three bounded tools for one persistent case."""

    def __init__(self, case_id: str, case: PersistentPodmanCase, transaction: AnswerTransaction, prompt_mode: PromptMode, limits: BrokerLimits = BrokerLimits()) -> None:
        self.case_id = case_id
        self.case = case
        self.transaction = transaction
        self.prompt_mode = prompt_mode
        self.limits = limits
        self._image_count = 0
        self._image_attempted = False
        self._successful_image_read = False
        self._pending_image_read = False
        self._audit: list[dict[str, object]] = []

    @property
    def audit(self) -> tuple[dict[str, object], ...]:
        return tuple(self._audit)

    def sandbox_exec(self, command: str) -> dict[str, object]:
        if not isinstance(command, str) or not command or len(command.encode("utf-8")) > self.limits.max_command_bytes:
            raise ValueError("command is empty or exceeds the configured bound")
        try:
            result = self.case.exec(command)
        except Exception:
            self._audit.append({"tool": "sandbox_exec", "case_id": self.case_id, "command": _safe_command(command), "outcome": "error"})
            raise
        response = {"stdout": _truncate_utf8(result.stdout, self.limits.max_output_bytes), "stderr": _truncate_utf8(result.stderr, self.limits.max_output_bytes), "exit_code": result.returncode}
        self._audit.append({"tool": "sandbox_exec", "case_id": self.case_id, "command": _safe_command(command), "outcome": "ok" if result.returncode == 0 else "exit_nonzero"})
        return response

    def read_generated_image(self, path: str) -> dict[str, str]:
        self._image_attempted = True
        if self.prompt_mode == "visualization-forbidden":
            self._audit.append({"tool": "read_generated_image", "case_id": self.case_id, "outcome": "policy_violation"})
            raise PolicyViolationError(VISUALIZATION_FORBIDDEN_ERROR)
        if self._image_count >= self.limits.max_images:
            raise ValueError("image count limit exceeded")
        if not isinstance(path, str) or len(path) == 0 or len(path) > 512 or path.startswith("/") or not re.fullmatch(r"[A-Za-z0-9._/-]+", path):
            raise ValueError("image path must be relative to /work")
        relative = path
        if "\\" in relative or any(part == ".." for part in Path(relative).parts):
            raise ValueError("image path contains traversal")
        data = _read_workspace_file(Path(self.case.request.workspace_dir), relative, self.limits.max_image_bytes)
        if len(data) > self.limits.max_image_bytes or data[:8] != b"\x89PNG\r\n\x1a\n":
            raise ValueError("image must be a bounded PNG")
        width, height = _png_dimensions(data)
        if width > self.limits.max_image_width or height > self.limits.max_image_height or width * height > self.limits.max_image_pixels:
            raise ValueError("image dimensions exceed configured bounds")
        _decode_png(data, width, height)
        self._pending_image_read = True
        return {"mime": "image/png", "data": base64.b64encode(data).decode("ascii")}

    def commit_image_read(self, *, delivered: bool) -> None:
        """Commit a validated image only after the controller accepts its reply."""
        if not self._pending_image_read:
            return
        self._pending_image_read = False
        if not delivered:
            return
        self._image_count += 1
        self._successful_image_read = True
        self._audit.append({"tool": "read_generated_image", "case_id": self.case_id, "outcome": "success"})

    def submit_answer(self, answer: bool | int) -> AnswerReceipt:
        if self.prompt_mode == "visualization-encouraged" and not self._successful_image_read:
            self._audit.append({"tool": "submit_answer", "case_id": self.case_id, "accepted": False, "outcome": "policy_violation"})
            raise PolicyViolationError(VISUALIZATION_REQUIRED_ERROR)
        receipt = self.transaction.submit(answer)
        self._audit.append({"tool": "submit_answer", "case_id": self.case_id, "accepted": receipt.accepted})
        return receipt

    @property
    def compliant(self) -> bool:
        return (self.prompt_mode == "visualization-forbidden" and not self._image_attempted) or (
            self.prompt_mode == "visualization-encouraged" and self._successful_image_read
        )

    def assert_compliant(self) -> None:
        if self.compliant:
            return
        raise PolicyViolationError(
            VISUALIZATION_FORBIDDEN_ERROR
            if self.prompt_mode == "visualization-forbidden"
            else VISUALIZATION_REQUIRED_ERROR
        )

    def dispatch(self, tool: str, arguments: dict[str, object]) -> object:
        routes: dict[str, Callable[..., object]] = {
            "sandbox_exec": self.sandbox_exec,
            "read_generated_image": self.read_generated_image,
            "submit_answer": self.submit_answer,
        }
        if tool not in routes:
            raise ValueError(f"unknown tool: {tool}")
        if tool == "read_generated_image" and self.prompt_mode == "visualization-forbidden":
            self._image_attempted = True
            self._audit.append({"tool": tool, "case_id": self.case_id, "outcome": "policy_violation"})
            raise PolicyViolationError(VISUALIZATION_FORBIDDEN_ERROR)
        expected = {"sandbox_exec": {"command"}, "read_generated_image": {"path"}, "submit_answer": {"answer"}}[tool]
        if set(arguments) != expected:
            raise ValueError("tool arguments do not match the exact schema")
        return routes[tool](**arguments)


def _png_dimensions(data: bytes) -> tuple[int, int]:
    if len(data) < 24 or data[12:16] != b"IHDR":
        raise ValueError("PNG header is malformed")
    width, height = struct.unpack(">II", data[16:24])
    if width == 0 or height == 0:
        raise ValueError("PNG dimensions must be positive")
    return width, height


def _decode_png(data: bytes, width: int, height: int) -> None:
    """Require a complete, natively decodable PNG without retaining pixel data."""
    try:
        offset = 8
        saw_iend = False
        while offset < len(data):
            if offset + 12 > len(data):
                raise ValueError("truncated PNG chunk")
            length = struct.unpack(">I", data[offset : offset + 4])[0]
            end = offset + 12 + length
            if end > len(data):
                raise ValueError("truncated PNG chunk")
            chunk_type = data[offset + 4 : offset + 8]
            chunk_data = data[offset + 8 : offset + 8 + length]
            expected_crc = struct.unpack(">I", data[offset + 8 + length : end])[0]
            if zlib.crc32(chunk_type + chunk_data) & 0xFFFFFFFF != expected_crc:
                raise ValueError("PNG chunk checksum failed")
            offset = end
            if chunk_type == b"IEND":
                saw_iend = True
                break
        if not saw_iend or offset != len(data):
            raise ValueError("PNG stream is incomplete")
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            with Image.open(BytesIO(data)) as image:
                if image.format != "PNG" or image.size != (width, height):
                    raise ValueError("image must be a valid PNG")
                image.verify()
            with Image.open(BytesIO(data)) as image:
                image.load()
    except Exception as error:
        raise ValueError("image must be a fully decodable PNG") from error


def _truncate_utf8(value: str, limit: int) -> str:
    return value.encode("utf-8", errors="replace")[:limit].decode("utf-8", errors="ignore")


def _safe_command(command: str) -> str:
    sanitized = re.sub(r"(?:^|\s)(?:/[^\s]+)", " <path>", command)
    sanitized = "".join(character if character.isprintable() else "?" for character in sanitized)
    return _truncate_utf8(sanitized, 256)


def _read_workspace_file(root: Path, relative: str, limit: int) -> bytes:
    """Read a workspace-relative file without following symlinks during lookup."""
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    root_fd = os.open(root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    descriptor = root_fd
    try:
        parts = [part for part in relative.split("/") if part]
        if not parts:
            raise ValueError("image path is empty")
        for part in parts[:-1]:
            next_descriptor = os.open(part, flags | getattr(os, "O_DIRECTORY", 0), dir_fd=descriptor)
            if descriptor != root_fd:
                os.close(descriptor)
            descriptor = next_descriptor
        try:
            final = os.open(parts[-1], flags, dir_fd=descriptor)
        except OSError as error:
            raise ValueError("image path is not a safe regular file") from error
        try:
            data = bytearray()
            while len(data) <= limit:
                chunk = os.read(final, min(65_536, limit + 1 - len(data)))
                if not chunk:
                    break
                data.extend(chunk)
            if len(data) > limit or not stat.S_ISREG(os.fstat(final).st_mode):
                raise ValueError("image is not a bounded regular file")
            return bytes(data)
        except OSError as error:
            raise ValueError("image path is not a safe regular file") from error
        finally:
            os.close(final)
    finally:
        if descriptor != root_fd:
            os.close(descriptor)
        os.close(root_fd)
