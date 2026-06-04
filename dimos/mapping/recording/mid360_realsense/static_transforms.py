"""Static transforms between the RealSense D435i frames and the Mid-360 odom frame.

Computes rigid transforms (RealSense color/gyro/depth -> Mid-360 IMU frame) for use when
recording, and renders every frame in Rerun as XYZ basis arrows plus simple boxes for the
camera and lidar bodies (self-contained: no mesh files needed).

Frame sources
-------------
RealSense D435i frame transforms are transcribed from the official
realsense2_description xacro (urdf/_d435.urdf.xacro + urdf/_d435i_imu_modules.urdf.xacro,
use_nominal_extrinsics=true).

Mid-360 geometry (manual): body is 65 x 65 x 60 mm; the point-cloud origin O lies on the
central vertical axis, ~47 mm above the base. The IMU chip is *not* on that axis.
- Livox Mid-360 User Manual (Dimensions + Coordinates):
  https://www.livoxtech.com/mid-360/downloads

FAST-LIO odometry is the IMU ("body") frame, NOT the lidar frame
---------------------------------------------------------------
hku-mars/FAST_LIO runs its EKF on the IMU state: publish_odometry() sets
child_frame_id = "body" and fills the pose from state_point (the IMU pose in the
gravity-aligned "camera_init" world frame). RGBpointBodyLidarToIMU() transforms lidar
points into the IMU frame via offset_R_L_I / offset_T_L_I, confirming "body" == IMU.
- https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp

The lidar-to-IMU extrinsic comes from the official Mid-360 config; it matches the local
dimos config (dimos/hardware/sensors/lidar/fastlio2/config/mid360.yaml):
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]   # lidar origin expressed in IMU frame
Flipping the sign gives the IMU position in lidar coords = (11.0, 23.29, -44.12) mm,
the value Livox publishes for the IMU in the point-cloud coordinate system. The large
y = 23.29 mm term is why the IMU sits near a corner of the body, not the center.
- https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml
- https://github.com/Livox-SDK/livox_ros_driver2/issues/139

So the odom frame FAST-LIO reports is `imu_frame` below, not `lidar_frame`.
"""

import argparse
import math
from pathlib import Path

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

CAMERA_ANGLE_UP = math.radians(10)

# Mid-360 box: pitched down from bottom_screw_frame, then offset back/up in that frame
BOX_PITCH_DOWN = math.radians(26) + CAMERA_ANGLE_UP
BOX_BACK = 0.085
BOX_UP = 0.037  # ~4cm up

ROOT = "d435i"

# Physical constants from _d435.urdf.xacro (meters)
CAM_HEIGHT = 0.025
DEPTH_PY = 0.0175
DEPTH_PZ = CAM_HEIGHT / 2
MOUNT_FROM_CENTER_OFFSET = 0.0149
GLASS_TO_FRONT = 0.1e-3
ZERO_DEPTH_TO_GLASS = 4.2e-3
MESH_X_OFFSET = MOUNT_FROM_CENTER_OFFSET - GLASS_TO_FRONT - ZERO_DEPTH_TO_GLASS

DEPTH_TO_INFRA1_OFFSET = 0.0
DEPTH_TO_INFRA2_OFFSET = -0.050
DEPTH_TO_COLOR_OFFSET = 0.015
IMU_XYZ = (-0.01174, -0.00552, 0.0051)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # X red, Y green, Z blue

# D435i body box (90 x 25 x 25 mm), centered on bottom_screw_frame in x/y and lifted in
# z so its bottom face sits on the screw plane (x fwd, y left, z up).
CAMERA_BOX_HALF_SIZES = (0.0125, 0.045, 0.0125)
CAMERA_BOX_CENTER = (0.0, 0.0, CAMERA_BOX_HALF_SIZES[2])
CAMERA_BOX_COLOR = [80, 160, 230]

BOX_HALF_SIZES = (0.0325, 0.0325, 0.030)  # Mid-360 body: 65 x 65 x 60 mm
BOX_COLOR = [230, 160, 40]

# Mid-360 internal frames (manual: point-cloud origin O ~47mm above base, on central axis).
# Box center is 30mm above base, so O sits +17mm along box +z.
LIDAR_ABOVE_BOX_CENTER = 0.017
# IMU position in point-cloud (lidar) coordinates, from Livox Mid-360 extrinsics.
IMU_IN_LIDAR = (0.011, 0.02329, -0.04412)

# (name, parent_name, translation_xyz, rpy) — parent None means attached to ROOT
FRAMES: list[tuple[str, str | None, tuple[float, float, float], tuple[float, float, float]]] = [
    ("bottom_screw_frame", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    # Flat world: the screw frame leveled out. The camera points up CAMERA_ANGLE_UP, so
    # pitching the screw frame down by that angle gives a gravity-flat frame at the screw.
    ("world", "bottom_screw_frame", (0.0, 0.0, 0.0), (0.0, CAMERA_ANGLE_UP, 0.0)),
    ("link", "bottom_screw_frame", (MESH_X_OFFSET, DEPTH_PY, DEPTH_PZ), (0.0, 0.0, 0.0)),
    ("depth_frame", "link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("depth_optical_frame", "depth_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra1_frame", "link", (0.0, DEPTH_TO_INFRA1_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra1_optical_frame", "infra1_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra2_frame", "link", (0.0, DEPTH_TO_INFRA2_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra2_optical_frame", "infra2_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("color_frame", "link", (0.0, DEPTH_TO_COLOR_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("color_optical_frame", "color_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("accel_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("accel_optical_frame", "accel_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("gyro_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("gyro_optical_frame", "gyro_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("box_pitch_frame", "bottom_screw_frame", (0.0, 0.0, 0.0), (0.0, BOX_PITCH_DOWN, 0.0)),
    ("box_center", "box_pitch_frame", (-BOX_BACK, 0.0, BOX_UP), (0.0, 0.0, 0.0)),
    ("lidar_frame", "box_center", (0.0, 0.0, LIDAR_ABOVE_BOX_CENTER), (0.0, 0.0, 0.0)),
    ("imu_frame", "lidar_frame", IMU_IN_LIDAR, (0.0, 0.0, 0.0)),
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


# RealSense frame -> Mid-360 IMU (odom) frame. The IMU frame is what FAST-LIO reports as
# odom. Use `.inverse()` for the Mid-360 -> RealSense direction, or swap to the
# *_optical_frame source if you need the image-optical convention.
REALSENSE_COLOR_FRAME_TO_MID360_IMU_FRAME = transform_between(
    "color_frame", "imu_frame", "realsense_color_frame", "mid360_imu_frame"
)
REALSENSE_GYRO_FRAME_TO_MID360_IMU_FRAME = transform_between(
    "gyro_frame", "imu_frame", "realsense_gyro_frame", "mid360_imu_frame"
)
REALSENSE_DEPTH_FRAME_TO_MID360_IMU_FRAME = transform_between(
    "depth_frame", "imu_frame", "realsense_depth_frame", "mid360_imu_frame"
)
# Optical convention (x-right, y-down, z-forward) — what image/pointcloud data uses.
REALSENSE_COLOR_OPTICAL_FRAME_TO_MID360_IMU_FRAME = transform_between(
    "color_optical_frame", "imu_frame", "realsense_color_optical_frame", "mid360_imu_frame"
)
# Mid-360 IMU (FAST-LIO odom) frame -> flat world frame at the camera screw. Invert for the
# world -> mid360 direction used to re-anchor FAST-LIO odometry while recording.
MID360_TO_WORLD = transform_between("imu_frame", "world", "mid360_imu_frame", "world")


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

    rr.init("mid360_realsense_frames", spawn=rrd_path is None)
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
        f"{paths['bottom_screw_frame']}/box",
        rr.Boxes3D(
            centers=[list(CAMERA_BOX_CENTER)],
            half_sizes=[list(CAMERA_BOX_HALF_SIZES)],
            colors=[CAMERA_BOX_COLOR],
        ),
    )
    rr.log(
        f"{paths['box_center']}/box",
        rr.Boxes3D(
            centers=[[0.0, 0.0, 0.0]], half_sizes=[list(BOX_HALF_SIZES)], colors=[BOX_COLOR]
        ),
    )

    if rrd_path is not None:
        print(f"wrote {rrd_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--axis-length", type=float, default=0.015, help="basis-vector length in meters"
    )
    parser.add_argument(
        "--rrd", type=Path, default=None, help="save to this .rrd instead of spawning the viewer"
    )
    args = parser.parse_args()

    for transform in (
        REALSENSE_COLOR_FRAME_TO_MID360_IMU_FRAME,
        REALSENSE_GYRO_FRAME_TO_MID360_IMU_FRAME,
        REALSENSE_DEPTH_FRAME_TO_MID360_IMU_FRAME,
        REALSENSE_COLOR_OPTICAL_FRAME_TO_MID360_IMU_FRAME,
        MID360_TO_WORLD,
    ):
        print(transform)
    render(args.axis_length, args.rrd)


if __name__ == "__main__":
    main()
