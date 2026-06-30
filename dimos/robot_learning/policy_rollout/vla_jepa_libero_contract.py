# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import cast

from dimos_runtime_protocol import ObservationFrame, ObservationKind
import numpy as np

from dimos.robot_learning.policy_rollout.evaluation import RuntimeObservationSample
from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    RobotPolicyContractDescription,
    RuntimeActionOutput,
)

VLA_JEPA_LIBERO_ACTION_SPACE_ID = "libero.ee_delta_6d_gripper.normalized.v1"


class VlaJepaLiberoRobotContract:
    """Hardcoded VLA-JEPA LIBERO policy IO contract for the v1 rollout gate."""

    def __init__(
        self,
        *,
        agentview_stream: str = "agentview",
        wrist_stream_candidates: Sequence[str] = ("eye_in_hand", "wrist", "robot0_eye_in_hand"),
        state_stream: str = "robot_state",
    ) -> None:
        self._agentview_stream = agentview_stream
        self._wrist_stream_candidates = tuple(wrist_stream_candidates)
        self._state_stream = state_stream

    def to_backend_batch(self, sample: RuntimeObservationSample) -> BackendBatch:
        frames = {frame.stream: frame for frame in sample.observations}
        payloads = _payload_mapping(sample.metadata)
        agentview = self._image_payload(frames, payloads, self._agentview_stream)
        wrist_stream = self._select_wrist_stream(frames)
        wrist = self._image_payload(frames, payloads, wrist_stream)
        state = self._state_vector(frames, sample.metadata)
        language = self._language(sample)

        return BackendBatch(
            payload={
                "observation.images.image": agentview,
                "observation.images.image2": wrist,
                "observation.state": state,
                "task": language,
            },
            metadata={
                "episode_id": sample.episode_id,
                "tick_id": sample.tick_id,
                "task_id": sample.task_id,
                "task_index": sample.task_index,
                "init_state_index": sample.init_state_index,
                "agentview_stream": self._agentview_stream,
                "wrist_stream": wrist_stream,
            },
        )

    def from_backend_output(self, output: BackendOutputEnvelope) -> RuntimeActionOutput:
        values = _flat_float_values(output.output)
        if len(values) != 7:
            raise ValueError(
                f"VLA-JEPA LIBERO action must have shape (7,), got {len(values)} values"
            )
        if not all(np.isfinite(values)):
            raise ValueError("VLA-JEPA LIBERO action contains non-finite values")
        if any(value < -1.0 or value > 1.0 for value in values):
            raise ValueError("VLA-JEPA LIBERO action must be within [-1, 1]")
        return RuntimeActionOutput(
            space_id=VLA_JEPA_LIBERO_ACTION_SPACE_ID,
            values=tuple(values),
            metadata={"backend_metadata": dict(output.metadata)},
        )

    def describe(self) -> RobotPolicyContractDescription:
        return RobotPolicyContractDescription(
            contract_type="vla_jepa_libero",
            input_description={
                "agentview_stream": self._agentview_stream,
                "wrist_stream_candidates": list(self._wrist_stream_candidates),
                "state_stream": self._state_stream,
                "image_shape": [3, 128, 128],
                "image_dtype": "float32",
                "image_value_range": [0.0, 1.0],
                "state_shape": [8],
                "language_source": "runtime_description.metadata.language",
            },
            output_description={
                "space_id": VLA_JEPA_LIBERO_ACTION_SPACE_ID,
                "shape": [7],
                "bounds": [-1.0, 1.0],
                "semantics": "relative 6D end-effector delta plus gripper",
            },
        )

    def _select_wrist_stream(self, frames: Mapping[str, ObservationFrame]) -> str:
        for stream in self._wrist_stream_candidates:
            if stream in frames:
                return stream
        raise ValueError(
            "missing VLA-JEPA LIBERO wrist/eye-in-hand image stream; "
            f"expected one of {self._wrist_stream_candidates}"
        )

    def _image_payload(
        self,
        frames: Mapping[str, ObservationFrame],
        payloads: Mapping[str, object],
        stream: str,
    ) -> np.ndarray:
        frame = frames.get(stream)
        if frame is None:
            raise ValueError(f"missing VLA-JEPA LIBERO image stream {stream!r}")
        if frame.kind != ObservationKind.IMAGE:
            raise ValueError(f"observation stream {stream!r} must be an image")
        payload = payloads.get(stream, frame.metadata.get("array"))
        if payload is None:
            raise ValueError(f"missing image payload for stream {stream!r}")
        image = np.asarray(payload)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"image stream {stream!r} must have HWC RGB shape")
        if image.dtype != np.uint8:
            raise ValueError(f"image stream {stream!r} must have dtype uint8")
        flipped = np.flip(image, axis=(0, 1)).copy()
        return np.transpose(flipped, (2, 0, 1)).astype(np.float32) / 255.0

    def _state_vector(
        self, frames: Mapping[str, ObservationFrame], metadata: Mapping[str, object]
    ) -> np.ndarray:
        frame = frames.get(self._state_stream)
        if frame is None:
            raise ValueError(f"missing VLA-JEPA LIBERO state stream {self._state_stream!r}")
        if frame.kind != ObservationKind.STATE:
            raise ValueError(f"observation stream {self._state_stream!r} must be state")
        state = metadata.get(
            "robot_state", frame.metadata.get("state", frame.metadata.get("values"))
        )
        if state is None:
            raise ValueError("missing VLA-JEPA LIBERO 8D robot state values")
        array = np.asarray(state, dtype=np.float32)
        if array.shape != (8,):
            raise ValueError(f"VLA-JEPA LIBERO robot state must have shape (8,), got {array.shape}")
        if not np.all(np.isfinite(array)):
            raise ValueError("VLA-JEPA LIBERO robot state contains non-finite values")
        return array

    def _language(self, sample: RuntimeObservationSample) -> str:
        language = sample.metadata.get(
            "language", sample.runtime_description.metadata.get("language")
        )
        if not isinstance(language, str) or not language.strip():
            raise ValueError("missing VLA-JEPA LIBERO task language")
        return language


def _payload_mapping(metadata: Mapping[str, object]) -> Mapping[str, object]:
    payloads = metadata.get("payloads", {})
    if not isinstance(payloads, Mapping):
        raise ValueError("sample metadata payloads must be a mapping")
    return cast("Mapping[str, object]", payloads)


def _flat_float_values(value: object) -> list[float]:
    converted = _as_numpy_compatible(value)
    array = np.asarray(converted, dtype=np.float32)
    if array.ndim == 2 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 1:
        raise ValueError(f"VLA-JEPA LIBERO action must be 1D or single-batch 2D, got {array.shape}")
    return [float(item) for item in array.tolist()]


def _as_numpy_compatible(value: object) -> object:
    detach = getattr(value, "detach", None)
    if callable(detach):
        value = detach()
    cpu = getattr(value, "cpu", None)
    if callable(cpu):
        value = cpu()
    numpy = getattr(value, "numpy", None)
    if callable(numpy):
        return numpy()
    return value
