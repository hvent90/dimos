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
from deployment import ExampleExternalDeclaration
import humanize

from dimos.core.core import rpc
from dimos.core.module import Module


class ExampleExternalRuntime(ExampleExternalDeclaration, Module):
    @rpc
    def start(self) -> None:
        super().start()
        self.requests.subscribe(self._reply_to_request)

    def _reply_to_request(self, name: str) -> None:
        self.replies.publish(self.dependency_report(name))

    @rpc
    def greet(self, name: str) -> str:
        return f"{self.config.greeting}, {name} from external runtime"

    @rpc
    def greet_with_helper(self, name: str) -> str:
        return f"{self.greet(name)}; {self.helper.help_greet(name)}"

    @rpc
    def dependency_report(self, name: str) -> str:
        formatted = humanize.intcomma(1234567)
        return f"external-only humanize formatted {formatted} for {name}"
