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

from collections.abc import Callable, Mapping
from contextlib import nullcontext
import importlib
from typing import Any, Protocol, cast

import numpy as np

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    JsonObject,
    PolicyBackendDescription,
)

_VLA_JEPA_POLICY_MODULE = "lerobot.policies.vla_jepa.modeling_vla_jepa"
_VLA_JEPA_PROCESSOR_MODULE = "lerobot.policies.vla_jepa.processor_vla_jepa"
_DEFAULT_CHECKPOINT = "lerobot/VLA-JEPA-LIBERO"


class _PolicyConfig(Protocol):
    device: str


class _LeRobotPolicy(Protocol):
    config: _PolicyConfig

    def eval(self) -> object: ...

    def reset(self) -> object: ...

    def select_action(self, batch: Mapping[str, object]) -> object: ...

    def predict_action_chunk(self, batch: Mapping[str, object]) -> object: ...


class _PolicyClass(Protocol):
    @classmethod
    def from_pretrained(cls, pretrained_name_or_path: str) -> _LeRobotPolicy: ...


class _NoGrad(Protocol):
    def __enter__(self) -> object: ...

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> object: ...


class _TorchModule(Protocol):
    def no_grad(self) -> _NoGrad: ...


class LeRobotBackend:
    """Optional in-process LeRobot policy backend for VLA-JEPA LIBERO."""

    def __init__(
        self,
        *,
        checkpoint_id: str = _DEFAULT_CHECKPOINT,
        device: str | None = None,
        use_action_chunk: bool = False,
        use_processors: bool = True,
        dataset_stats: JsonObject | None = None,
    ) -> None:
        self._checkpoint_id = checkpoint_id
        self._device = device
        self._use_action_chunk = use_action_chunk
        self._use_processors = use_processors
        self._dataset_stats = dataset_stats
        self._policy: _LeRobotPolicy | None = None
        self._preprocessor: Callable[[Mapping[str, object]], Mapping[str, object]] | None = None
        self._postprocessor: Callable[[object], object] | None = None
        self._policy_class_name: str | None = None
        self._processor_source: str | None = None

    def initialize(self) -> None:
        if self._policy is not None:
            return
        policy_cls = self._load_policy_class()
        policy = policy_cls.from_pretrained(self._checkpoint_id)
        self._policy_class_name = _qualified_name(policy)
        self._configure_device(policy)
        policy.eval()
        self._policy = policy
        self._prepare_processors(policy)

    def reset_episode(self) -> None:
        policy = self._require_policy()
        policy.reset()

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
        policy = self._require_policy()
        backend_batch: Mapping[str, object] = _tensorized_preprocessor_input(batch.payload)
        if self._preprocessor is not None:
            backend_batch = self._preprocessor(backend_batch)
        with self._no_grad():
            output = (
                policy.predict_action_chunk(backend_batch)
                if self._use_action_chunk
                else policy.select_action(backend_batch)
            )
        if self._postprocessor is not None:
            output = self._postprocessor(output)
        return BackendOutputEnvelope(
            output=output,
            metadata={
                "backend_type": "lerobot",
                "checkpoint_id": self._checkpoint_id,
                "inference_method": "predict_action_chunk"
                if self._use_action_chunk
                else "select_action",
                "output_shape": _shape_of(output),
                "batch_metadata": dict(batch.metadata),
            },
        )

    def close(self) -> None:
        self._policy = None
        self._preprocessor = None
        self._postprocessor = None

    def describe(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(
            backend_type="lerobot",
            checkpoint_id=self._checkpoint_id,
            device=self._resolved_device(),
            policy_class=self._policy_class_name,
            supports_episode_reset=True,
            metadata={
                "policy_family": "vla_jepa",
                "use_action_chunk": self._use_action_chunk,
                "use_processors": self._use_processors,
                "processor_source": self._processor_source,
            },
        )

    def _load_policy_class(self) -> _PolicyClass:
        try:
            module = importlib.import_module(_VLA_JEPA_POLICY_MODULE)
        except ImportError as exc:
            raise RuntimeError(
                "Install LeRobot from GitHub main to use LeRobotBackend for "
                "lerobot/VLA-JEPA-LIBERO. The PyPI lerobot package may not include "
                "the VLA-JEPA policy yet. Example: uv run --with "
                "git+https://github.com/huggingface/lerobot.git ..."
            ) from exc
        policy_cls = getattr(module, "VLAJEPAPolicy", None)
        if policy_cls is None:
            raise RuntimeError("LeRobot VLAJEPAPolicy class was not found")
        return cast("_PolicyClass", policy_cls)

    def _configure_device(self, policy: _LeRobotPolicy) -> None:
        if self._device is None:
            return
        config = getattr(policy, "config", None)
        if config is not None:
            cast("_PolicyConfig", config).device = self._device
        to_device = getattr(policy, "to", None)
        if callable(to_device):
            to_device(self._device)

    def _prepare_processors(self, policy: _LeRobotPolicy) -> None:
        if not self._use_processors:
            return
        if self._prepare_checkpoint_processors(policy):
            return
        self._prepare_manual_vla_jepa_processors(policy)

    def _prepare_checkpoint_processors(self, policy: _LeRobotPolicy) -> bool:
        try:
            module = importlib.import_module("lerobot.policies.factory")
        except ImportError:
            return False
        factory = getattr(module, "make_pre_post_processors", None)
        if not callable(factory):
            return False
        device = self._resolved_device()
        try:
            processors = factory(
                policy_cfg=policy.config,
                pretrained_path=self._checkpoint_id,
                preprocessor_overrides={"device_processor": {"device": str(device)}}
                if device is not None
                else None,
            )
        except (AttributeError, ImportError, TypeError, RuntimeError):
            return False
        self._install_processors(processors)
        self._processor_source = "checkpoint"
        return True

    def _prepare_manual_vla_jepa_processors(self, policy: _LeRobotPolicy) -> None:
        try:
            module = importlib.import_module(_VLA_JEPA_PROCESSOR_MODULE)
        except ImportError:
            return
        factory = getattr(module, "make_vla_jepa_pre_post_processors", None)
        if not callable(factory):
            return
        processors = factory(policy.config, dataset_stats=self._dataset_stats)
        self._install_processors(processors)
        self._processor_source = "manual_vla_jepa"

    def _install_processors(self, processors: object) -> None:
        preprocessor, postprocessor = cast("tuple[object, object]", processors)
        if callable(preprocessor):
            self._preprocessor = cast(
                "Callable[[Mapping[str, object]], Mapping[str, object]]", preprocessor
            )
        if callable(postprocessor):
            self._postprocessor = cast("Callable[[object], object]", postprocessor)

    def _require_policy(self) -> _LeRobotPolicy:
        self.initialize()
        if self._policy is None:
            raise RuntimeError("LeRobot policy did not initialize")
        return self._policy

    def _no_grad(self) -> _NoGrad:
        try:
            torch_module = cast("_TorchModule", importlib.import_module("torch"))
        except ImportError:
            return cast("_NoGrad", nullcontext())
        return torch_module.no_grad()

    def _resolved_device(self) -> str | None:
        if self._device is not None:
            return self._device
        if self._policy is None:
            return None
        config = getattr(self._policy, "config", None)
        device = getattr(config, "device", None)
        return str(device) if device is not None else None


def _qualified_name(value: object) -> str:
    cls = value.__class__
    return f"{cls.__module__}.{cls.__qualname__}"


def _tensorized_preprocessor_input(payload: Mapping[str, object]) -> dict[str, object]:
    prepared: dict[str, object] = {}
    for key, value in payload.items():
        if isinstance(value, np.ndarray):
            prepared[key] = _to_torch_tensor(key, value)
        else:
            prepared[key] = value
    return prepared


def _to_torch_tensor(key: str, value: np.ndarray) -> object:
    torch_module = cast(Any, importlib.import_module("torch"))
    array = value
    if key.startswith("observation.images.") and array.ndim == 3:
        if array.shape[-1] in (1, 3):
            array = np.transpose(array, (2, 0, 1))
        if array.dtype == np.uint8:
            array = array.astype(np.float32) / 255.0
    return torch_module.as_tensor(array, dtype=torch_module.float32)


def _shape_of(value: object) -> list[int]:
    shape = getattr(value, "shape", None)
    if shape is not None:
        return [int(dim) for dim in shape]
    if isinstance(value, (list, tuple)):
        if value and isinstance(value[0], (list, tuple)):
            return [len(value), len(value[0])]
        return [len(value)]
    return []
