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

"""G1 arm teleop adapter.

Bridges the babylon viewer's ``HumanoidControlSpec`` to the
``ControlCoordinator``'s ``servo_arms`` task. The viewer's per-joint slider
HUD calls ``set_arm_joint(name, position)`` via RPC; this module owns the
14-joint arm target vector and publishes the full vector on the
``joint_command`` stream (transport-mapped to ``/g1/joint_command``) every
time. The ``servo_arms`` task expects a complete 14-vector target — its
contract is "give me a target, I'll hold it" — so the adapter is the place
that knows how to assemble that target from per-joint slider events.

The current target is seeded from the latest joint_state when the first
``set_arm_joint`` arrives (so we don't snap the other 13 joints away from
where they actually are). Subsequent calls mutate one slot and republish.

Joint names exposed to the viewer are the short form ("left_shoulder_pitch"),
matching the convention the babylon slider HUD uses. The ``g1/`` hardware-id
prefix is added internally when publishing, and stripped from incoming
joint_state names.
"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any
import xml.etree.ElementTree as ET

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_HW_ID = "g1"
_URDF_PATH = Path(__file__).resolve().parent / "g1.urdf"

# Canonical arm-joint short names, left arm then right arm (matches
# ``make_humanoid_joints("g1")[15:]`` ordering).
_ARM_JOINT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)
_FULL_JOINT_NAMES: tuple[str, ...] = tuple(f"{_HW_ID}/{n}" for n in _ARM_JOINT_NAMES)


def _load_arm_limits() -> dict[str, tuple[float, float]]:
    """Parse arm joint limits from the G1 URDF. Maps short name → (lo, hi)."""
    tree = ET.parse(_URDF_PATH)
    root = tree.getroot()
    limits: dict[str, tuple[float, float]] = {}
    for short in _ARM_JOINT_NAMES:
        full = short + "_joint"
        for joint in root.iter("joint"):
            if joint.get("name") != full:
                continue
            limit_el = joint.find("limit")
            if limit_el is None:
                raise RuntimeError(f"G1 URDF: {full} has no <limit>")
            lo = float(limit_el.get("lower", "0"))
            hi = float(limit_el.get("upper", "0"))
            limits[short] = (lo, hi)
            break
        else:
            raise RuntimeError(f"G1 URDF: joint {full} not found")
    return limits


class G1ArmTeleop(Module):
    """Owns the 14-joint arm target vector and publishes it on demand.

    Ports:
        joint_state (In[JointState]): coordinator-published full-body state,
            used to seed the target from the real arm pose on first interaction.
        joint_command (Out[JointState]): full 14-joint arm target, routed by
            the coordinator to the ``servo_arms`` task.
    """

    joint_state: In[JointState]
    joint_command: Out[JointState]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._limits = _load_arm_limits()
        # Latest observed arm pose, keyed by short name. Updated from
        # joint_state callbacks.
        self._state_lock = threading.Lock()
        self._latest_pose: dict[str, float] | None = None
        # The current 14-joint target we'll publish. None until first
        # ``set_arm_joint`` (or ``release_arms``) — we don't want to publish
        # a target before the user asks for one, otherwise we'd race the
        # servo_arms task's default_positions on startup.
        self._target: list[float] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.joint_state.subscribe(self._on_joint_state)
        logger.info("G1ArmTeleop ready (%d arm joints, limits from URDF)", len(self._limits))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest arm pose so we can seed our target from reality."""
        if not msg.name or not msg.position:
            return
        pose: dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position, strict=False):
            # Coordinator names are "g1/<short>"; tolerate plain "<short>" too.
            short = name.split("/", 1)[1] if "/" in name else name
            if short in self._limits:
                pose[short] = float(pos)
        if not pose:
            return
        with self._state_lock:
            self._latest_pose = pose

    def _seed_target(self) -> list[float]:
        """Build a 14-vector target from the latest observed arm pose.

        Falls back to zeros (the servo task's default_positions) if no
        joint_state has been observed yet — the user clicking a slider
        before state arrives is unlikely but cheap to handle.
        """
        with self._state_lock:
            pose = dict(self._latest_pose) if self._latest_pose is not None else {}
        return [pose.get(name, 0.0) for name in _ARM_JOINT_NAMES]

    def _publish_target(self) -> None:
        assert self._target is not None
        msg = JointState(name=list(_FULL_JOINT_NAMES), position=list(self._target))
        self.joint_command.publish(msg)

    @rpc
    def arm_joint_limits(self) -> list[tuple[str, float, float]]:
        """(short_name, lower_rad, upper_rad) for each of the 14 arm joints,
        in left-then-right URDF order."""
        return [(name, *self._limits[name]) for name in _ARM_JOINT_NAMES]

    @rpc
    def set_arm_joint(self, name: str, position: float) -> bool:
        """Drive one arm joint. ``name`` is the short form (e.g.,
        ``left_shoulder_pitch``). Position is clamped to URDF limits. The
        other 13 joints stay at their last commanded value (or the current
        real pose on the first call), so we publish a complete 14-vector
        target every time — what the servo_arms task expects."""
        limit = self._limits.get(name)
        if limit is None:
            logger.warning("G1ArmTeleop: unknown arm joint %r", name)
            return False
        if self._target is None:
            self._target = self._seed_target()
        lo, hi = limit
        idx = _ARM_JOINT_NAMES.index(name)
        self._target[idx] = max(lo, min(hi, float(position)))
        self._publish_target()
        return True

    @rpc
    def release_arms(self) -> bool:
        """Send all 14 arm joints back to neutral (zero pose)."""
        self._target = [0.0] * len(_ARM_JOINT_NAMES)
        self._publish_target()
        return True


g1_arm_teleop = G1ArmTeleop.blueprint

__all__ = ["G1ArmTeleop", "g1_arm_teleop"]
