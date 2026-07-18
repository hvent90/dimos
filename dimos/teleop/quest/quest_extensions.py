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

"""Quest teleop module extensions and subclasses.

Available subclasses:
    - ArmTeleopModule: Per-hand press-and-hold engage (X/A hold to track), task name routing
    - TwistTeleopModule: Outputs Twist instead of PoseStamped
    - VideoArmTeleopModule: ArmTeleopModule + JPEG frames pushed to the Quest over /ws
    - Go2TeleopModule: Thumbstick → Twist velocity for the Go2 + camera over /ws
"""

import asyncio
from typing import Any

from fastapi import WebSocket
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs.UInt32 import UInt32
from dimos.teleop.quest.quest_teleop_module import QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.quest.quest_types import Buttons, Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


async def _ws_send_jpeg(ws: WebSocket, data: bytes) -> None:
    try:
        await ws.send_bytes(data)
    except Exception:
        # Client closed or write failed — drop the frame; the base /ws
        # disconnect handler evicts the dead client.
        pass


def _push_bytes(module: QuestTeleopModule, data: bytes) -> None:
    """Broadcast raw bytes to all of module's connected /ws clients.

    Runs on the RX thread; sends are scheduled on the asyncio loop captured by
    QuestTeleopModule when the first client connected. Used for both JPEG frames
    and small LCM-encoded control messages (the client dispatches inbound blobs
    by LCM fingerprint, falling back to JPEG).
    """
    loop = module._ws_loop
    if loop is None:
        return
    with module._clients_lock:
        clients = tuple(module._connected_clients)
    if not clients:
        return
    for ws in clients:
        asyncio.run_coroutine_threadsafe(_ws_send_jpeg(ws, data), loop)


def _push_jpeg(module: QuestTeleopModule, msg: Image, quality: int) -> None:
    """JPEG-encode an Image and push it to all of module's connected /ws clients."""
    # Skip the encode entirely if nobody is listening.
    if module._ws_loop is None:
        return
    with module._clients_lock:
        if not module._connected_clients:
            return
    try:
        jpeg = msg.to_jpeg_bytes(quality=quality)
    except Exception:
        logger.exception("Failed to encode camera frame")
        return
    _push_bytes(module, jpeg)


class TwistTeleopConfig(QuestTeleopConfig):
    """Configuration for TwistTeleopModule."""

    linear_scale: float = 1.0
    angular_scale: float = 1.0


# Example implementation to show how to extend QuestTeleopModule for different teleop behaviors and outputs.
class TwistTeleopModule(QuestTeleopModule):
    """Quest teleop that outputs TwistStamped instead of PoseStamped.

    Config:
        - linear_scale: Scale factor for linear (position) values. Default 1.0.
        - angular_scale: Scale factor for angular (orientation) values. Default 1.0.

    Outputs:
        - left_twist: TwistStamped (linear + angular velocity)
        - right_twist: TwistStamped (linear + angular velocity)
        - buttons: Buttons (inherited)
    """

    config: TwistTeleopConfig

    left_twist: Out[TwistStamped]
    right_twist: Out[TwistStamped]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Convert PoseStamped to TwistStamped, apply scaling, and publish."""
        twist = TwistStamped(
            ts=output_msg.ts,
            frame_id=output_msg.frame_id,
            linear=output_msg.position * self.config.linear_scale,
            angular=output_msg.orientation.to_euler() * self.config.angular_scale,
        )
        if hand == Hand.LEFT:
            self.left_twist.publish(twist)
        else:
            self.right_twist.publish(twist)


class ArmTeleopConfig(QuestTeleopConfig):
    """Configuration for ArmTeleopModule.

    Attributes:
        task_names: Mapping of Hand -> coordinator task name. Used to set
            frame_id on output PoseStamped so the coordinator routes each
            hand's commands to the correct TeleopIKTask.
    """

    task_names: dict[str, str] = Field(default_factory=dict)


class ArmTeleopModule(QuestTeleopModule):
    """Quest teleop with per-hand press-and-hold engage and task name routing.

    Each controller's primary button (X for left, A for right)
    engages that hand while held, disengages on release.

    When task_names is configured, output PoseStamped messages have their
    frame_id set to the task name, enabling the coordinator to route
    each hand's commands to the correct TeleopIKTask.

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
    """

    config: ArmTeleopConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        self._task_names: dict[Hand, str] = {
            Hand[k.upper()]: v for k, v in self.config.task_names.items()
        }

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        """Stamp frame_id with task name and publish."""
        task_name = self._task_names.get(hand)
        if task_name:
            output_msg = PoseStamped(
                position=output_msg.position,
                orientation=output_msg.orientation,
                ts=output_msg.ts,
                frame_id=task_name,
            )
        super()._publish_msg(hand, output_msg)

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        """Publish Buttons with analog triggers packed into bits 16-29."""
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self.teleop_buttons.publish(buttons)


class DrawArmTeleopModule(ArmTeleopModule):
    """ArmTeleopModule where the grip button ALSO engages the controller pose stream.

    The base module only publishes ``*_controller_output`` while a hand is
    engaged, and only the primary (X/A) button engages. For the VR draw-a-line
    feature the operator sketches with the *grip* button (arm stays frozen —
    a downstream TrajectoryReplayModule gates the poses away from the arm), so
    the pose stream must also flow while grip is held, otherwise there are no
    poses to record.

    This overrides engage so a hand engages when **primary OR grip** is held.
    The engage-time reference pose is captured on whichever edge fires first, so
    deltas are relative to where the gesture began. The trigger is left free for
    its usual gripper role. Live X/A teleop behaves exactly as before.

    Also relays a downstream replay-progress signal (permille [0,1000]) to the
    headset over ``/ws`` so the web app can consume the drawn line as the arm
    retraces it.

    Inputs:
        - replay_progress: In[UInt32] (optional — wire to a TrajectoryReplayModule)
    """

    replay_progress: In[UInt32]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.replay_progress.subscribe(self._on_progress)))

    def _on_progress(self, msg: UInt32) -> None:
        """Relay replay progress to the headset as an LCM-encoded UInt32.

        The web client dispatches inbound blobs by LCM fingerprint (UInt32 →
        progress) and otherwise treats them as JPEG frames.
        """
        try:
            _push_bytes(self, msg.lcm_encode())
        except Exception:
            logger.exception("Failed to push replay progress to headset")

    def _handle_engage(self) -> None:
        for hand in Hand:
            controller = self._controllers.get(hand)
            if controller is None:
                continue
            # Either gesture holds the hand engaged; release both to disengage.
            active = controller.primary or controller.grip > 0.5
            if active:
                if not self._is_engaged[hand]:
                    self._engage(hand)
            else:
                if self._is_engaged[hand]:
                    self._disengage(hand)


class VideoArmTeleopConfig(ArmTeleopConfig):
    """Configuration for VideoArmTeleopModule."""

    video_jpeg_quality: int = 70


class VideoArmTeleopModule(ArmTeleopModule):
    """ArmTeleopModule + camera frames pushed to the Quest as JPEG over /ws.

    Subscribes to color_image, JPEG-encodes each frame, and broadcasts raw
    JPEG bytes to every connected /ws client as a binary message. The client
    decodes via createObjectURL and uploads to a WebGL texture.

    Inputs:
        - color_image: In[Image] (required — wire to a camera output)

    Outputs:
        - left_controller_output: PoseStamped (inherited)
        - right_controller_output: PoseStamped (inherited)
        - buttons: Buttons (inherited)
    """

    config: VideoArmTeleopConfig

    color_image: In[Image]

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)


class Go2TeleopConfig(QuestTeleopConfig):
    """Configuration for Go2TeleopModule."""

    linear_speed: float = 0.5  # m/s at full stick deflection
    angular_speed: float = 0.8  # rad/s at full stick deflection
    deadzone: float = 0.1
    video_jpeg_quality: int = 70


class Go2TeleopModule(QuestTeleopModule):
    """Quest teleop for the Unitree Go2: thumbstick driving + camera in the headset.

    Velocity is derived from the controller thumbsticks as each Joy message
    arrives (left stick → forward/strafe, right stick → yaw) and published on
    cmd_vel for GO2Connection.move. The Go2 camera (color_image) is JPEG-encoded
    and pushed to the headset over /ws. A deadzone suppresses stick drift.

    Inputs:
        - color_image: In[Image] (wire to the Go2 camera output)

    Outputs:
        - cmd_vel: Twist (base velocity command)
    """

    config: Go2TeleopConfig

    color_image: In[Image]
    cmd_vel: Out[Twist]

    def _deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.config.deadzone else v

    def _on_joy_bytes(self, data: bytes) -> None:
        super()._on_joy_bytes(data)
        with self._lock:
            left = self._controllers.get(Hand.LEFT)
            right = self._controllers.get(Hand.RIGHT)
        twist = Twist()
        twist.linear = Vector3(0.0, 0.0, 0.0)
        twist.angular = Vector3(0.0, 0.0, 0.0)
        if left is not None:
            twist.linear.x = -self._deadzone(left.thumbstick.y) * self.config.linear_speed
            twist.linear.y = -self._deadzone(left.thumbstick.x) * self.config.linear_speed
        if right is not None:
            twist.angular.z = -self._deadzone(right.thumbstick.x) * self.config.angular_speed
        self.cmd_vel.publish(twist)

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)

    @rpc
    def stop(self) -> None:
        # Send one zero Twist so the base halts if teleop dies mid-motion.
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("Failed to publish stop Twist")
        super().stop()
