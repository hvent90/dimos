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

import argparse
from pathlib import Path
import pickle
import signal
import time
from typing import cast

from dimos.core.deployment.models import ExternalModule, LaunchEnvelope
from dimos.core.module import Module


def _resolve_class(ref: str) -> type[object]:
    module_name, name = ref.split(":", 1)
    module = __import__(module_name, fromlist=[name])
    resolved = getattr(module, name)
    if not isinstance(resolved, type):
        raise TypeError(f"{ref} did not resolve to a class")
    return resolved


def _load_envelope(path: Path) -> LaunchEnvelope:
    with path.open("rb") as f:
        envelope = pickle.load(f)
    if not isinstance(envelope, LaunchEnvelope):
        raise TypeError("Launch envelope file did not contain a LaunchEnvelope")
    return envelope


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-envelope", required=True)
    args = parser.parse_args()
    envelope = _load_envelope(Path(args.launch_envelope))
    declaration = envelope.module_class
    if not issubclass(declaration, ExternalModule):
        raise TypeError(f"{declaration.__name__} is not an ExternalModule declaration")
    metadata = envelope.metadata
    runtime_class = _resolve_class(metadata.runtime_ref)
    if not issubclass(runtime_class, declaration) or not issubclass(runtime_class, Module):
        raise TypeError(f"{runtime_class.__name__} must subclass {declaration.__name__} and Module")
    module_class = cast("type[Module]", runtime_class)
    module = module_class(**dict(envelope.kwargs))
    if module.rpc is None:
        raise RuntimeError(f"{module_class.__name__} did not start an RPC backend")
    module.rpc.serve_module_rpc(module, name=declaration.__name__)
    stopped = False

    def _stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True
        module.stop()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    while not stopped:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
