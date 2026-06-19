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

"""Hosted teleop blueprints (WebRTC transport)."""

from pathlib import Path

from dimos.constants import STATE_DIR
from dimos.control.blueprints.teleop import coordinator_teleop_xarm7
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport, LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.quest_hosted.hosted_extensions import (
    HostedArmTeleopModule,
    HostedTwistTeleopModule,
)
from dimos.teleop.quest_hosted.state_bridge import TeleopStateBridge
from dimos.teleop.utils.recorder import TeleopRecorder, TeleopRecorderConfig

# Single XArm7 teleop via the hosted (WebRTC) client. Pass `--simulation` to
# run the coordinator inside MuJoCo, omit it for real hardware.
teleop_hosted_xarm7 = (
    autoconnect(
        HostedArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
        coordinator_teleop_xarm7,
    )
    .transports(
        {
            ("right_controller_output", PoseStamped): LCMTransport.spec(
                "/coordinator/cartesian_command", PoseStamped
            ),
            ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
        }
    )
    .global_config(rerun_open="none")
)


# viewer="none" drops the rerun window (operator gets video over WebRTC, so the
# robot-side rerun view is unwanted here).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")


# Hosted teleop as a pure transport swap — no teleop module wrapper. The
# browser's keyboard/VR view sends LCM TwistStamped on cmd_unreliable; the
# transport decodes it straight onto the go2 cmd_vel stream (commands arrive
# as sent: normalized [-1, 1], no speed rescaling). The camera stream feeds
# the session's WebRTC video track via CloudflareVideoTransport (same
# provider/PeerConnection), and robot → operator telemetry can ride
# CloudflareTransport.spec("state_reliable_back", ...) the same way.
#
# Clock-sync pings are answered inside BrokerProvider. TeleopStateBridge
# republishes operator video_stats as Out[VideoStats] (recorder picks it up)
# and pushes robot_telemetry (command-plane latency/jitter/loss) back to the
# operator HUD on state_reliable_back, measured from a raw tap on the command
# wire — no TwistStamped change.
#
# Run:  dimos run teleop-hosted-go2-transport -o transports.broker.api_key=dtk_live_...
#       (or TRANSPORTS__BROKER__API_KEY=dtk_live_... in env; robot identity is
#       derived from the key, override with transports.broker.robot_id if needed)
# then connect from https://teleop.dimensionalos.com (keyboard view).
teleop_hosted_go2_transport = (
    autoconnect(
        unitree_go2_basic,
        TeleopStateBridge.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): CloudflareTransport.spec("cmd_unreliable", TwistStamped),
            ("color_image", Image): CloudflareVideoTransport.spec(),
            ("state_json", bytes): CloudflareTransport.spec("state_reliable"),
            ("telemetry_out", bytes): CloudflareTransport.spec("state_reliable_back"),
            # Raw tap on the command wire for command-plane stats (reads the
            # Header's stamp+seq off the bytes). Independent subscriber from
            # cmd_vel above — same channel, separate decode.
            ("cmd_raw", bytes): CloudflareTransport.spec("cmd_unreliable"),
            # Recorder tap on the command wire: when hosted-teleop-recorder is
            # composed, its cmd_vel_stamped port subscribes to the SAME channel
            # — independent typed decode, stamped with the browser's ts so the
            # report's timing math works. Unused (harmless) when no recorder.
            ("cmd_vel_stamped", TwistStamped): CloudflareTransport.spec(
                "cmd_unreliable", TwistStamped
            ),
        }
    )
    .global_config(viewer="none")
)


HOSTED_RECORDINGS_DIR = STATE_DIR / "hosted_teleop" / "recordings"


class HostedTeleopRecorderConfig(TeleopRecorderConfig):
    # Same generic recorder, just defaulting recordings into the hosted dir.
    db_path: str | Path = HOSTED_RECORDINGS_DIR / "recording_hosted.db"


class HostedTeleopRecorder(TeleopRecorder):
    """Generic ``TeleopRecorder`` defaulting to the hosted recordings dir.

    Ports + per-run timestamping are inherited; this only changes the default
    output path. Compose at the CLI::

        dimos run teleop-hosted-xarm7 hosted-teleop-recorder
        dimos run teleop-hosted-go2   hosted-teleop-recorder
    """

    config: HostedTeleopRecorderConfig


__all__ = [
    "HostedTeleopRecorder",
    "HostedTeleopRecorderConfig",
    "teleop_hosted_go2",
    "teleop_hosted_go2_transport",
    "teleop_hosted_xarm7",
]
