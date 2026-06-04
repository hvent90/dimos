"""Static transforms between the Go2 body, its front camera, and the Mid-360 odom frame.

Computes the rigid mounts used while recording (Mid-360 IMU -> base_link, base_link ->
camera optical) and renders every frame in Rerun as XYZ basis arrows plus a simple box
for the robot body (self-contained: no mesh/URDF files needed).

Mount geometry (measured on the physical rig)
---------------------------------------------
- base_link -> front_camera: 32.7cm forward, ~4.3cm up (URDF front_camera mount).
- front_camera -> mid360_link: lidar is 3.2cm back, 12cm up, pitched 44 deg down.
- front_camera -> camera_optical: the standard ROS optical rotation (x-right, y-down,
  z-forward).

FAST-LIO odometry is the Mid-360 IMU ("body") frame
---------------------------------------------------
hku-mars/FAST_LIO runs its EKF on the IMU state and publishes child_frame_id="body" (the
IMU pose in the gravity-aligned world frame), so the odom FAST-LIO reports is the Mid-360
frame tracked here as `mid360_link`.
- https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp
"""

import argparse
import math
from pathlib import Path

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

ROOT = "go2"

MID360_PITCH_DOWN = math.radians(44.0)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # X red, Y green, Z blue

# Approximate Go2 trunk box (~65 x 31 x 15 cm), centered on base_link. The front face lands
# at the front_camera mount (x ~= 0.327). Adjust if you want the full leg-span bounding box.
GO2_BODY_HALF_SIZES = (0.325, 0.155, 0.075)
GO2_BODY_CENTER = (0.0, 0.0, 0.0)
GO2_BODY_COLOR = [120, 120, 130]

# (name, parent_name, translation_xyz, rpy) — parent None means attached to ROOT
FRAMES: list[tuple[str, str | None, tuple[float, float, float], tuple[float, float, float]]] = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_camera", "base_link", (0.32715, -0.00003, 0.04297), (0.0, 0.0, 0.0)),
    ("mid360_link", "front_camera", (-0.032, 0.0, 0.12), (0.0, MID360_PITCH_DOWN, 0.0)),
    ("camera_optical", "front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
]

PARENT_OF: dict[str, str | None] = {name: parent for name, parent, _, _ in FRAMES}
EDGE_OF: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {
    name: (translation, rpy) for name, _parent, translation, rpy in FRAMES
}


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis rpy -> rotation matrix (Rz @ Ry @ Rx)."""
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    rot_x = np.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
    rot_y = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
    rot_z = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def pose_relative_to_root(name: str) -> Transform:
    """Compose the frame edges from ROOT down to ``name`` (root -> name)."""
    chain: list[str] = []
    cursor: str | None = name
    while cursor is not None:
        chain.append(cursor)
        cursor = PARENT_OF[cursor]
    pose: Transform | None = None
    for frame_name in reversed(chain):
        translation, rpy = EDGE_OF[frame_name]
        edge = Transform(
            translation=Vector3(*translation), rotation=Quaternion.from_euler(Vector3(*rpy))
        )
        pose = edge if pose is None else pose + edge
    assert pose is not None
    return pose


def transform_between(source: str, target: str, frame_id: str, child_frame_id: str) -> Transform:
    """Static transform source -> target (frame_id=source, child_frame_id=target)."""
    result = pose_relative_to_root(source).inverse() + pose_relative_to_root(target)
    result.frame_id = frame_id
    result.child_frame_id = child_frame_id
    return result


BASE_TO_FRONT_CAMERA = transform_between("base_link", "front_camera", "base_link", "front_camera")
BASE_TO_MID360 = transform_between("base_link", "mid360_link", "base_link", "mid360_link")
# Mid-360 IMU (FAST-LIO odom) frame -> robot base. Compose with world->mid360 from odom to
# anchor recorded observations to base_link.
MID360_TO_BASE = transform_between("mid360_link", "base_link", "mid360_link", "base_link")
# base_link -> camera optical frame (x-right, y-down, z-forward) for anchoring images.
BASE_TO_CAMERA_OPTICAL = transform_between(
    "base_link", "camera_optical", "base_link", "camera_optical"
)


def entity_path(name: str) -> str:
    chain = [name]
    cursor = PARENT_OF[name]
    while cursor is not None:
        chain.append(cursor)
        cursor = PARENT_OF[cursor]
    chain.append(ROOT)
    return "/".join(reversed(chain))


def render(axis_length: float, rrd_path: Path | None) -> None:
    import rerun as rr

    rr.init("go2_mid360_frames", spawn=rrd_path is None)
    if rrd_path is not None:
        rr.save(str(rrd_path))

    rr.log(ROOT, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    paths = {name: entity_path(name) for name in PARENT_OF}

    axes = np.eye(3) * axis_length
    for name, _parent, translation, rpy in FRAMES:
        rr.log(
            paths[name],
            rr.Transform3D(translation=list(translation), mat3x3=rpy_to_matrix(*rpy)),
            rr.Arrows3D(
                vectors=axes, origins=np.zeros((3, 3)), colors=AXIS_COLORS, labels=["X", "Y", "Z"]
            ),
        )
        rr.log(
            f"{paths[name]}/label",
            rr.Points3D([[0.0, 0.0, 0.0]], labels=[name], show_labels=True, radii=axis_length / 15),
        )

    rr.log(
        f"{paths['base_link']}/body",
        rr.Boxes3D(
            centers=[list(GO2_BODY_CENTER)],
            half_sizes=[list(GO2_BODY_HALF_SIZES)],
            colors=[GO2_BODY_COLOR],
        ),
    )

    if rrd_path is not None:
        print(f"wrote {rrd_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--axis-length", type=float, default=0.1, help="basis-vector length in meters"
    )
    parser.add_argument(
        "--rrd", type=Path, default=None, help="save to this .rrd instead of spawning the viewer"
    )
    args = parser.parse_args()

    for transform in (BASE_TO_FRONT_CAMERA, BASE_TO_MID360, MID360_TO_BASE, BASE_TO_CAMERA_OPTICAL):
        print(transform)
    render(args.axis_length, args.rrd)


if __name__ == "__main__":
    main()
