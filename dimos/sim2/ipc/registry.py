# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run-scoped discovery for sim2 channels."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any

from dimos.sim2.ipc.abi import ChannelDescriptor

RUN_ID_ENV = "DIMOS_RUN_ID"


def current_run_id() -> str:
    return os.environ.get(RUN_ID_ENV, f"local-{os.getpid()}")


def shared_memory_name(run_id: str, sim_id: str, robot_id: str, generation: str) -> str:
    digest = hashlib.sha256(f"{run_id}\0{sim_id}\0{robot_id}\0{generation}".encode()).hexdigest()[
        :20
    ]
    return f"dms2_{digest}"


def control_socket_path(run_id: str, sim_id: str, generation: str) -> Path:
    digest = hashlib.sha256(f"{run_id}\0{sim_id}\0{generation}".encode()).hexdigest()[:20]
    return Path(tempfile.gettempdir()) / f"dms2_{digest}.sock"


class SimRegistry:
    def __init__(self, run_id: str | None = None, root: Path | None = None) -> None:
        self.run_id = run_id or current_run_id()
        self.root = root or Path(tempfile.gettempdir()) / "dimos-sim2"

    def manifest_path(self, sim_id: str) -> Path:
        digest = hashlib.sha256(f"{self.run_id}\0{sim_id}".encode()).hexdigest()[:20]
        return self.root / f"{digest}.json"

    def publish(
        self,
        sim_id: str,
        generation: str,
        descriptors: dict[str, ChannelDescriptor],
        *,
        socket_path: str | None = None,
    ) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.manifest_path(sim_id)
        value: dict[str, Any] = {
            "run_id": self.run_id,
            "sim_id": sim_id,
            "generation": generation,
            "socket_path": socket_path,
            "channels": {
                robot_id: descriptor.to_dict()
                for robot_id, descriptor in sorted(descriptors.items())
            },
        }
        temporary = path.with_suffix(f".{os.getpid()}.tmp")
        temporary.write_text(json.dumps(value, sort_keys=True))
        temporary.replace(path)
        return path

    def resolve(self, sim_id: str, robot_id: str) -> ChannelDescriptor:
        path = self.manifest_path(sim_id)
        if not path.exists():
            raise FileNotFoundError(f"sim2 manifest not found for sim_id='{sim_id}'")
        value = json.loads(path.read_text())
        if value.get("run_id") != self.run_id or value.get("sim_id") != sim_id:
            raise ValueError(f"sim2 manifest identity mismatch: {path}")
        try:
            raw = value["channels"][robot_id]
        except KeyError as exc:
            raise KeyError(
                f"robot '{robot_id}' is not registered in simulation '{sim_id}'"
            ) from exc
        descriptor = ChannelDescriptor.from_dict(raw)
        if descriptor.generation != value.get("generation"):
            raise ValueError(f"stale channel generation for robot '{robot_id}'")
        return descriptor

    def resolve_socket(self, sim_id: str) -> Path:
        path = self.manifest_path(sim_id)
        if not path.exists():
            raise FileNotFoundError(f"sim2 manifest not found for sim_id='{sim_id}'")
        value = json.loads(path.read_text())
        socket_path = value.get("socket_path")
        if not socket_path:
            raise ValueError(f"simulation '{sim_id}' has no control socket")
        return Path(socket_path)

    def remove(self, sim_id: str, generation: str) -> None:
        path = self.manifest_path(sim_id)
        if not path.exists():
            return
        try:
            value = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            return
        if value.get("generation") == generation:
            path.unlink(missing_ok=True)
