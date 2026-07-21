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

from unittest.mock import patch

from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule


def test_quest_web_server_is_initialized_during_start() -> None:
    with (
        patch("dimos.teleop.quest.quest_teleop_module.RobotWebInterface") as web_interface,
        patch.object(QuestTeleopModule, "_setup_routes") as setup_routes,
        patch.object(QuestTeleopModule, "_start_server") as start_server,
        patch.object(QuestTeleopModule, "_start_control_loop"),
    ):
        module = QuestTeleopModule(server_port=9443)
        try:
            web_interface.assert_not_called()
            assert module._web_server is None

            module.start()

            web_interface.assert_called_once_with(host="0.0.0.0", port=9443)
            assert module._web_server is web_interface.return_value
            setup_routes.assert_called_once_with()
            start_server.assert_called_once_with()
        finally:
            module.stop()
