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

"""Errors for the optional VAMP manipulation planning backend."""


class VampDependencyError(ImportError):
    """Raised when the VAMP backend is selected without the optional dependency."""

    def __init__(self) -> None:
        super().__init__(
            "VAMP planning backend requires the optional 'vamp-planner' dependency. "
            "Install the DimOS VAMP extra or install vamp-planner in this environment."
        )


class UnsupportedWorldCapabilityError(NotImplementedError):
    """Raised when a world backend does not natively support a requested capability."""

    def __init__(self, backend: str, capability: str) -> None:
        super().__init__(f"World backend '{backend}' does not support capability: {capability}")
