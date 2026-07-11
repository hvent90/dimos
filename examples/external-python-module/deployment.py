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
from pathlib import Path
import time

from pydantic import Field

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.deployment.models import DeploymentSpec, ExternalModule, LocalPythonPackage
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out


class ExampleExternalConfig(ModuleConfig):
    greeting: str = Field(default="hello")


class ExampleHelper(Module):
    @rpc
    def help_greet(self, name: str) -> str:
        return f"helper saw {name}"


class ExampleExternalDeclaration(ExternalModule):
    config: ExampleExternalConfig
    requests: In[str]
    replies: Out[str]
    helper: ExampleHelper

    @rpc
    def greet(self, name: str) -> str: ...

    @rpc
    def greet_with_helper(self, name: str) -> str: ...

    @rpc
    def dependency_report(self, name: str) -> str: ...


class ExampleClient(Module):
    heavy: ExampleExternalDeclaration
    requests: Out[str]
    replies: In[str]

    def __init__(self, **kwargs: object) -> None:
        self._last_reply: str | None = None
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self.replies.subscribe(self._record_reply)

    def _record_reply(self, reply: str) -> None:
        self._last_reply = reply

    @rpc
    def call_external_dependency(self, name: str) -> str:
        return self.heavy.dependency_report(name)

    @rpc
    def roundtrip_stream(self, name: str, timeout_s: float = 3.0) -> str:
        self._last_reply = None
        self.requests.publish(name)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._last_reply is not None:
                return self._last_reply
            time.sleep(0.01)
        raise TimeoutError("Timed out waiting for external stream reply")


deployment_spec = DeploymentSpec(
    blueprint=autoconnect(
        ExampleHelper.blueprint(),
        ExampleClient.blueprint(),
        ExampleExternalDeclaration.blueprint(greeting="hi"),
    ),
    external={
        ExampleExternalDeclaration: LocalPythonPackage(
            package_root=Path(__file__).parent,
            declaration=ExampleExternalDeclaration,
            runtime_ref="example_external.runtime:ExampleExternalRuntime",
        )
    },
)
