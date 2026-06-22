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

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import cv2
import numpy as np
from pydantic import Field
import reactivex as rx
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import (
    OPTICAL_ROTATION,
    DepthCameraConfig,
    DepthCameraHardware,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import perception
from dimos.utils.reactive import backpressure
from dimos.visualization.vis_module import vis_module

if TYPE_CHECKING:
    import pyrealsense2 as rs  # type: ignore[import-not-found,import-untyped]


def default_base_transform() -> Transform:
    """Default identity transform for camera mounting."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class RealSenseCameraConfig(ModuleConfig, DepthCameraConfig):
    width: int = 848
    height: int = 480
    fps: int = 15
    camera_name: str = "camera"
    base_frame_id: str = "base_link"
    base_transform: Transform | None = Field(default_factory=default_base_transform)
    align_depth_to_color: bool = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    enable_imu: bool = True
    pointcloud_fps: float = 5.0
    camera_info_fps: float = 1.0
    serial_number: str | None = None


class RealSenseCamera(DepthCameraHardware, Module, perception.DepthCamera):
    config: RealSenseCameraConfig
    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    imu: Out[Imu]

    @property
    def _camera_link(self) -> str:
        return f"{self.config.camera_name}_link"

    @property
    def _color_frame(self) -> str:
        return f"{self.config.camera_name}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.config.camera_name}_color_optical_frame"

    @property
    def _depth_frame(self) -> str:
        return f"{self.config.camera_name}_depth_frame"

    @property
    def _depth_optical_frame(self) -> str:
        return f"{self.config.camera_name}_depth_optical_frame"

    @property
    def _imu_frame(self) -> str:
        return f"{self.config.camera_name}_imu_optical_frame"

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._pipeline: rs.pipeline | None = None
        self._profile: rs.pipeline_profile | None = None
        self._align: rs.align | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._color_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._depth_scale: float = 0.001
        self._color_to_depth_extrinsics: rs.extrinsics | None = None
        # Pointcloud generation state
        self._latest_color_img: Image | None = None
        self._latest_depth_img: Image | None = None
        self._pointcloud_lock = threading.Lock()
        # IMU state
        self._imu_pipeline: rs.pipeline | None = None
        self._depth_to_imu_extrinsics: rs.extrinsics | None = None
        self._latest_accel: Vector3 | None = None

    @rpc
    def start(self) -> None:
        import pyrealsense2 as rs

        self._pipeline = rs.pipeline()
        config = rs.config()

        if self.config.serial_number:
            config.enable_device(self.config.serial_number)

        config.enable_stream(
            rs.stream.color,
            self.config.width,
            self.config.height,
            rs.format.bgr8,
            self.config.fps,
        )

        if self.config.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.config.width,
                self.config.height,
                rs.format.z16,
                self.config.fps,
            )

        self._profile = self._pipeline.start(config)

        if self.config.enable_depth:
            depth_sensor = self._profile.get_device().first_depth_sensor()
            self._depth_scale = depth_sensor.get_depth_scale()

        if self.config.align_depth_to_color and self.config.enable_depth:
            self._align = rs.align(rs.stream.color)

        self._build_camera_info()
        self._get_extrinsics()

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        if self.config.enable_pointcloud and self.config.enable_depth:
            interval_sec = 1.0 / self.config.pointcloud_fps
            self.register_disposable(
                backpressure(rx.interval(interval_sec)).subscribe(
                    on_next=lambda _: self._generate_pointcloud(),
                    on_error=lambda e: print(f"Pointcloud error: {e}"),
                )
            )

        interval_sec = 1.0 / self.config.camera_info_fps
        self.register_disposable(
            rx.interval(interval_sec).subscribe(
                on_next=lambda _: self._publish_camera_info(),
                on_error=lambda e: print(f"CameraInfo error: {e}"),
            )
        )

        if self.config.enable_imu:
            self._start_imu()

    def _start_imu(self) -> None:
        import pyrealsense2 as rs

        imu_pipeline = rs.pipeline()
        imu_config = rs.config()
        if self.config.serial_number:
            imu_config.enable_device(self.config.serial_number)

        try:
            # Explicit IMU rates required: without an fps librealsense falls back to
            # accel @ 63 Hz, which D4xx firmware doesn't offer -> "Couldn't resolve requests".
            imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
            imu_pipeline.start(imu_config, self._on_imu_frame)
        except RuntimeError as error:
            print(f"RealSense IMU unavailable, disabling IMU stream: {error}")
            return

        self._imu_pipeline = imu_pipeline

        if self._profile is not None and self.config.enable_depth:
            depth_stream = self._profile.get_stream(rs.stream.depth)
            # The accel profile must come from the device's extrinsics graph, not the
            # separately-started IMU pipeline -- cross-pipeline profiles aren't linked and
            # get_extrinsics_to() raises "Requested extrinsics are not available!".
            accel_stream = next(
                profile
                for sensor in self._profile.get_device().query_sensors()
                for profile in sensor.get_stream_profiles()
                if profile.stream_type() == rs.stream.accel
            )
            self._depth_to_imu_extrinsics = depth_stream.get_extrinsics_to(accel_stream)

    def _on_imu_frame(self, frame: rs.frame) -> None:
        if frame.is_frameset():
            for sub_frame in frame.as_frameset():
                self._handle_motion_frame(sub_frame)
        else:
            self._handle_motion_frame(frame)

    def _handle_motion_frame(self, frame: rs.frame) -> None:
        import pyrealsense2 as rs

        motion = frame.as_motion_frame()
        if not motion:
            return

        motion_data = motion.get_motion_data()
        stream_type = motion.get_profile().stream_type()

        if stream_type == rs.stream.accel:
            self._latest_accel = Vector3(motion_data.x, motion_data.y, motion_data.z)
        elif stream_type == rs.stream.gyro and self._latest_accel is not None:
            # Gyro drives publishing, paired with the most recent accel sample.
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(motion_data.x, motion_data.y, motion_data.z),
                    linear_acceleration=self._latest_accel,
                    frame_id=self._imu_frame,
                )
            )

    def _publish_camera_info(self) -> None:
        ts = time.time()
        if self._color_camera_info:
            self._color_camera_info.ts = ts
            self.camera_info.publish(self._color_camera_info)
        if self._depth_camera_info:
            self._depth_camera_info.ts = ts
            self.depth_camera_info.publish(self._depth_camera_info)

    def _build_camera_info(self) -> None:
        import pyrealsense2 as rs

        if self._profile is None:
            return

        # Color camera info
        color_stream = self._profile.get_stream(rs.stream.color).as_video_stream_profile()
        color_intrinsics = color_stream.get_intrinsics()
        self._color_camera_info = self._intrinsics_to_camera_info(
            color_intrinsics, self._color_optical_frame
        )

        # Depth camera info
        if self.config.enable_depth:
            if self.config.align_depth_to_color:
                # When aligned to color, depth uses color intrinsics and frame
                self._depth_camera_info = self._intrinsics_to_camera_info(
                    color_intrinsics, self._color_optical_frame
                )
            else:
                depth_stream = self._profile.get_stream(rs.stream.depth).as_video_stream_profile()
                depth_intrinsics = depth_stream.get_intrinsics()
                self._depth_camera_info = self._intrinsics_to_camera_info(
                    depth_intrinsics, self._depth_optical_frame
                )

    def _intrinsics_to_camera_info(self, intrinsics: rs.intrinsics, frame_id: str) -> CameraInfo:
        import pyrealsense2 as rs

        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy

        K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        D = list(intrinsics.coeffs) if intrinsics.coeffs else []

        distortion_model = {
            rs.distortion.none: "",
            rs.distortion.modified_brown_conrady: "plumb_bob",
            rs.distortion.inverse_brown_conrady: "plumb_bob",
            rs.distortion.ftheta: "equidistant",
            rs.distortion.brown_conrady: "plumb_bob",
            rs.distortion.kannala_brandt4: "equidistant",
        }.get(intrinsics.model, "")

        return CameraInfo(
            height=intrinsics.height,
            width=intrinsics.width,
            distortion_model=distortion_model,
            D=D,
            K=K,
            P=P,
            frame_id=frame_id,
        )

    def _get_extrinsics(self) -> None:
        import pyrealsense2 as rs

        if self._profile is None or not self.config.enable_depth:
            return

        depth_stream = self._profile.get_stream(rs.stream.depth)
        color_stream = self._profile.get_stream(rs.stream.color)
        self._color_to_depth_extrinsics = color_stream.get_extrinsics_to(depth_stream)

    def _extrinsics_to_transform(
        self,
        extrinsics: rs.extrinsics,
        frame_id: str,
        child_frame_id: str,
        ts: float,
    ) -> Transform:
        rotation_matrix = np.array(extrinsics.rotation).reshape(3, 3)
        quat = Rotation.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
        return Transform(
            translation=Vector3(*extrinsics.translation),
            rotation=Quaternion(quat[0], quat[1], quat[2], quat[3]),
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            ts=ts,
        )

    def _capture_loop(self) -> None:
        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except (RuntimeError, AttributeError):
                # Pipeline stopped or None - exit loop
                break

            ts = time.time()

            if self._align is not None:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.config.enable_depth else None

            # Process color
            color_img = None
            if color_frame:
                color_data = np.asanyarray(color_frame.get_data())
                color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
                color_img = Image(
                    data=color_data,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                self.color_image.publish(color_img)

            # Process depth
            depth_img = None
            if depth_frame:
                depth_data = np.asanyarray(depth_frame.get_data())
                # When aligned, depth is in color optical frame
                depth_frame_id = (
                    self._color_optical_frame
                    if self.config.align_depth_to_color
                    else self._depth_optical_frame
                )
                depth_img = Image(
                    data=depth_data,
                    format=ImageFormat.DEPTH16,
                    frame_id=depth_frame_id,
                    ts=ts,
                )
                self.depth_image.publish(depth_img)

            # Store latest images for pointcloud generation
            if self.config.enable_pointcloud and color_img is not None and depth_img is not None:
                with self._pointcloud_lock:
                    self._latest_color_img = color_img
                    self._latest_depth_img = depth_img

            # Publish TF
            self._publish_tf(ts)

    def _publish_tf(self, ts: float) -> None:
        transforms = []

        # base_link -> camera_link (user-provided mounting transform)
        if self.config.base_transform is not None:
            base_to_camera = Transform(
                translation=self.config.base_transform.translation,
                rotation=self.config.base_transform.rotation,
                frame_id=self.config.base_frame_id,
                child_frame_id=self._camera_link,
                ts=ts,
            )
            transforms.append(base_to_camera)

        # camera_link -> camera_depth_frame (identity, depth is at camera_link origin)
        camera_link_to_depth = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=self._camera_link,
            child_frame_id=self._depth_frame,
            ts=ts,
        )
        transforms.append(camera_link_to_depth)

        # camera_depth_frame -> camera_depth_optical_frame
        depth_to_depth_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=OPTICAL_ROTATION,
            frame_id=self._depth_frame,
            child_frame_id=self._depth_optical_frame,
            ts=ts,
        )
        transforms.append(depth_to_depth_optical)

        color_tf = self._extrinsics_to_transform(
            self._color_to_depth_extrinsics,
            self._camera_link,
            self._color_frame,
            ts,
        )
        # Invert the transform since extrinsics are color->depth
        color_tf = color_tf.inverse()
        color_tf.frame_id = self._camera_link
        color_tf.child_frame_id = self._color_frame
        color_tf.ts = ts
        transforms.append(color_tf)

        # camera_color_frame -> camera_color_optical_frame
        color_to_color_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=OPTICAL_ROTATION,
            frame_id=self._color_frame,
            child_frame_id=self._color_optical_frame,
            ts=ts,
        )
        transforms.append(color_to_color_optical)

        # camera_link -> imu (physical motion-sensor extrinsics)
        if self._depth_to_imu_extrinsics is not None:
            imu_tf = self._extrinsics_to_transform(
                self._depth_to_imu_extrinsics,
                self._camera_link,
                self._imu_frame,
                ts,
            )
            transforms.append(imu_tf)

        self.tf.publish(*transforms)

    def _generate_pointcloud(self) -> None:
        """Generate and publish pointcloud from latest images (called by rx.interval)."""
        with self._pointcloud_lock:
            color_img = self._latest_color_img
            depth_img = self._latest_depth_img

        if color_img is None or depth_img is None or self._color_camera_info is None:
            return

        try:
            pcd = PointCloud2.from_rgbd(
                color_image=color_img,
                depth_image=depth_img,
                camera_info=self._color_camera_info,
                depth_scale=self._depth_scale,
            )
            pcd = pcd.voxel_downsample(0.005)
            self.pointcloud.publish(pcd)
        except Exception as e:
            print(f"Pointcloud generation error: {e}")

    @rpc
    def stop(self) -> None:
        self._running = False

        if self._imu_pipeline:
            try:
                self._imu_pipeline.stop()
            except Exception:
                pass  # Pipeline might already be stopped
            self._imu_pipeline = None

        # Stop pipeline first to unblock wait_for_frames()
        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass  # Pipeline might already be stopped
            self._pipeline = None

        # Now join the thread (should exit quickly since pipeline is stopped)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                # Force thread termination by clearing reference
                self._thread = None

        self._profile = None
        self._align = None
        self._color_to_depth_extrinsics = None
        self._depth_to_imu_extrinsics = None
        self._latest_accel = None
        self._latest_color_img = None
        self._latest_depth_img = None
        super().stop()

    @rpc
    def get_color_camera_info(self) -> CameraInfo | None:
        return self._color_camera_info

    @rpc
    def get_depth_camera_info(self) -> CameraInfo | None:
        return self._depth_camera_info

    @rpc
    def get_depth_scale(self) -> float:
        return self._depth_scale


def _color_camera_info_to_rerun(camera_info: CameraInfo) -> object:
    # Re-parent the pinhole onto the color image entity + its optical frame so
    # the color image renders inside the camera frustum in the 3D view.
    return camera_info.to_rerun(
        image_topic="world/color_image",
        optical_frame="camera_color_optical_frame",
    )


def main() -> None:
    blueprint = autoconnect(
        RealSenseCamera.blueprint(enable_pointcloud=True, pointcloud_fps=5.0),
        vis_module(
            "rerun",
            rerun_config={"visual_override": {"world/camera_info": _color_camera_info_to_rerun}},
        ),
    )

    ModuleCoordinator.build(blueprint).loop()


if __name__ == "__main__":
    main()
