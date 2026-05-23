# Pointcloud coloring

For every lidar frame we need the closest-in-time camera frame so that, with
intrinsics + extrinsics, we can project each point into image space and read
back a colour. Lidar runs at ~7Hz, the camera at ~14Hz, and they're captured
independently — so step one is a streaming temporal alignment.

```python session=coloring
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.data import get_data

store = SqliteStore(path=get_data("go2_hongkong_office.db"))
lidar = store.streams.lidar
color_image = store.streams.color_image
print(lidar.summary())
print(color_image.summary())
```

```results
Stream("lidar"): 957 items, 2026-05-14 10:15:50 — 2026-05-14 10:18:17 (146.4s)
Stream("color_image"): 1984 items, 2026-05-14 10:15:52 — 2026-05-14 10:18:17 (144.9s)
```

`Stream.align` pairs each primary observation with the nearest one from
`other` within `tolerance` seconds. Streams iterate in ts order on both sides
and the matching is a bounded two-pointer merge — no full materialization,
no per-pair queries.

```python session=coloring
aligned = lidar.align(color_image, tolerance=0.05)
print(aligned.summary())
```

```results
Stream("lidar") | order_by(ts) -> FnIterTransformer(fn=_align): 932 items, 2026-05-14 10:15:52 — 2026-05-14 10:18:17 (144.9s)
```

Each output observation's `data` is a namedtuple keyed by source-stream name —
fully addressable both ways:

```python session=coloring
pair = aligned.first().data
print(f"lidar @ {pair.lidar.ts:.3f}  ↔  image @ {pair.color_image.ts:.3f}")
print(f"Δt = {(pair.color_image.ts - pair.lidar.ts) * 1000:.1f} ms")
print(f"index access works too: pair[0] is pair.lidar -> {pair[0] is pair.lidar}")
```

```results
lidar @ 1778753752.548  ↔  image @ 1778753752.551
Δt = 2.5 ms
index access works too: pair[0] is pair.lidar -> True
```

## Projecting 3D points into the image

For coloring we need a `CameraInfo` (intrinsics + distortion model) plus a
camera pose in the same frame as the points we'll project. The Go2's front
fisheye is an equidistant Kannala-Brandt model — calibration ships with the
repo as a YAML.

```python session=coloring
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.perception.pointcloud.projection import Camera
from dimos.robot.unitree.go2.connection import _camera_info_static

# Package-data lookup via importlib.resources — CWD-independent.
info = _camera_info_static()
camera = Camera(info=info, pose=Pose())  # identity pose: "points are in camera frame"
print(f"sensor: {info.width}x{info.height}, model={info.distortion_model}")
print(f"K[0,0]={info.K[0]:.1f}  K[1,1]={info.K[4]:.1f}  cx={info.K[2]:.1f}  cy={info.K[5]:.1f}")
```

```results
sensor: 1280x720, model=equidistant
K[0,0]=797.5  K[1,1]=796.5  cx=643.5  cy=349.3
```

Forward projection takes `(N,3)` points and returns `(pixels: (N,2), valid: (N,))`.
Invalid = behind the camera or projected outside the image bounds.

```python session=coloring
import numpy as np

# Synthetic points in the camera optical frame (z forward, x right, y down):
# a target ahead, two off-axis, one behind, one way out of frame.
pts = np.array([
    [0.0, 0.0, 3.0],     # straight ahead at 3m       -> hits (cx, cy)
    [0.3, -0.1, 2.0],    # slightly right + up        -> in-frame
    [-0.4, 0.2, 4.0],    # slightly left + down       -> in-frame
    [0.0, 0.0, -1.0],    # behind camera              -> invalid
    [5.0, 0.0, 1.0],     # 79° off-axis right         -> outside image
])
pixels, valid = camera.project(pts)
for p, (u, v), ok in zip(pts, pixels, valid):
    label = f"({u:7.1f}, {v:7.1f})" if ok else "  invalid"
    print(f"{p.tolist()!s:32}  ->  {label}")
```

```results
[0.0, 0.0, 3.0]                   ->  (  643.5,   349.3)
[0.3, -0.1, 2.0]                  ->  (  762.0,   309.9)
[-0.4, 0.2, 4.0]                  ->  (  564.2,   388.9)
[0.0, 0.0, -1.0]                  ->    invalid
[5.0, 0.0, 1.0]                   ->    invalid
```

Back-projection turns a pixel into a `Ray(origin, direction)`. The center
pixel's ray should point along +z (the optical axis) since this camera is
at the origin with identity orientation.

```python session=coloring
ray = camera.ray(info.K[2], info.K[5])  # ray through (cx, cy)
print(f"origin    = {ray.origin}")
print(f"direction = {ray.direction.round(3)}")
print(f"|dir|     = {np.linalg.norm(ray.direction):.6f}")
```

```results
origin    = [0. 0. 0.]
direction = [0. 0. 1.]
|dir|     = 1.000000
```

Real coloring still needs one more thing: a static `T_camera_lidar` extrinsic
so we can express each lidar point in the camera frame before `project()`.
That goes into the coloring transform itself (next step), which takes the
aligned `(lidar, color_image)` pairs and emits a colored pointcloud.

## Coloring a global map (memory2 transform)

The same projection works batch-style as a memory2 transform: pair each lidar
frame with the nearest image (`.align`), color the points using the image's
own optical-frame pose, then accumulate the colored frames into one global
map via `VoxelMapTransformer`. Because the recording stores each
`color_image.pose` as the camera optical frame in world coordinates (and the
lidar points are already in world), we don't need any robot-specific
`base→optical` extrinsic — the per-frame inverse of the image's own pose
is the world→optical transform we hand to `color_pointcloud`.

```python session=coloring
import numpy as np
import open3d as o3d
import open3d.core as o3c

from dimos.mapping.voxels import VoxelMapTransformer
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.lidar_color_module import color_pointcloud

# Drop pixels within this many of the image edge — fisheye distortion is
# unreliable near the rim, so a generous margin keeps colors honest.
BORDER_MARGIN = 80


def color_frame(obs):
    pair = obs.data  # AlignedPair(lidar, color_image)
    # The image carries its own camera-optical pose in world; invert -> world->optical.
    T_world_optical = Transform.from_pose("camera_optical", pair.color_image.pose_stamped)
    T_optical_world = -T_world_optical
    positions, colors = color_pointcloud(
        points_lidar=pair.lidar.data.points_f32(),  # already in world
        image=pair.color_image.data,
        camera_info=info,
        T_camera_lidar=T_optical_world.to_matrix(),
        border_margin=BORDER_MARGIN,
    )
    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3c.Tensor(positions, dtype=o3c.float32)
    pcd.point["colors"]    = o3c.Tensor((colors.astype(np.float32) / 255.0), dtype=o3c.float32)
    return obs.derive(data=PointCloud2(pointcloud=pcd, ts=pair.lidar.ts, frame_id="world"))

colored_global = (
    lidar.align(color_image, tolerance=0.05)
         .map(color_frame)
         .transform(VoxelMapTransformer(device="CPU:0", emit_every=0))
         .last()
         .data
)
t = colored_global.pointcloud_tensor
pts = t.point["positions"].numpy()
rgb = t.point["colors"].numpy()
sampled = ~np.all(np.abs(rgb - 128 / 255) < 0.01, axis=1)
print(f"{len(pts)} voxels — {sampled.sum()} with sampled colors, {(~sampled).sum()} outside any FOV")
```

```results
12:01:15.182 [inf][dimos/mapping/voxels.py       ] VoxelGrid using device: CPU:0
[1;33m[Open3D WARNING] Creating from an empty legacy PointCloud.[0;m
93419 voxels — 89223 with sampled colors, 4196 outside any FOV
```

Top-down scatter so you can see the colors line up with the scene:

```python session=coloring output=none
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("Agg")

fig, ax = plt.subplots(figsize=(10, 10), facecolor="black")
ax.scatter(pts[:, 0], pts[:, 1], c=rgb, s=0.5, marker=".")
ax.set_aspect("equal")
ax.set_facecolor("black")
ax.axis("off")
plt.savefig(
    "assets/colored_global_top_down.png",
    facecolor="black", dpi=120, bbox_inches="tight", pad_inches=0,
)
plt.close()
```

![output](assets/colored_global_top_down.png)

For an interactive 3D view of the same map, run [`demo_rerun.py`](./demo_rerun.py)
next to this doc — same pipeline but ends in
`Space().add(colored_global).to_rerun(...)`, which spawns the rerun viewer
with the colored cloud loaded. `PointCloud2.to_rerun()` honours the per-voxel
colors stored in the tensor, so no extra plumbing is needed.
