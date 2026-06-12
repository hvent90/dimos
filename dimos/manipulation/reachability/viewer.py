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

"""One-shot viser viewer for capability maps.

The view is the **body-frame workspace**: the arm's actual reachable
volume in pelvis coordinates, rendered as a point cloud colored
red→green by *dexterity* (fraction of approach angles reachable per
cell: red = one way in, green = approach from anywhere). An opaque
voxel style is available for a solid volume reading.

The IK ghost poses a **rigid** kinematic model. The real G1 arms are
compliant and sag a few cm under gravity at manipulation PD gains —
gravity-feedforward compensation belongs in the mink control task, not
this viewer.

Interactive extras:

- **IK target gizmo** — drag a 6-DOF target around; a mink QP poses the
  G1 URDF's arm to reach it live (self-collision-avoiding, same
  collision semantics as map construction), and the status line reports
  the IK result next to the map's prediction for the same pose.
- **Slice planes** — a vertical plane (adjustable yaw) and a horizontal
  plane (adjustable height) showing dexterity cross-sections in scene.

CLI::

    python -m dimos.manipulation.reachability.viewer \\
        --map ~/Desktop/g1_reachability/g1_left_capability.npz \\
        --map ~/Desktop/g1_reachability/g1_right_capability.npz
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from dimos.manipulation.reachability.capability_map import CapabilityMap
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_REPO_ROOT = Path(__file__).parents[3]
_G1_URDF = _REPO_ROOT / "data" / "g1_urdf" / "g1.urdf"
_EMPTY_GRAY = (45, 45, 55)


# ----------------------------------------------------------------------
# Pure geometry builders (unit-tested without a server)


def body_point_cloud(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[np.ndarray, np.ndarray]:
    """(points, dexterity) at occupied body-frame cell centers."""
    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    iz, ix, iy = np.nonzero(keep)
    if len(iz) == 0:
        return np.empty((0, 3)), np.empty(0)
    centers = (np.arange(params.n_xy) + 0.5) * params.cell - params.r_xy
    z_centers = (np.arange(params.n_z) + 0.5) * params.cell + params.z_min
    points = np.stack([centers[ix], centers[iy], z_centers[iz]], axis=1)
    return points, dexterity[iz, ix, iy]


def body_voxel_mesh(
    cap: CapabilityMap, min_dexterity: float, min_count: int = 1
) -> tuple[Any, int]:
    """Trimesh box-mesh of body-frame cells, vertex-colored by dexterity.

    Returns (mesh | None, n_voxels). ``min_dexterity`` in [0, 1] selects the
    dexterous core; 0 shows everything reachable.
    """
    import trimesh

    params = cap.params
    dexterity = cap.body_dexterity()
    keep = (cap.body_counts >= min_count) & (dexterity >= min_dexterity)
    n_voxels = int(keep.sum())
    if n_voxels == 0:
        return None, 0

    # (z, x, y) → (x, y, z) voxel matrix.
    matrix = keep.transpose(1, 2, 0)
    import matplotlib

    colors = np.zeros((*matrix.shape, 4), dtype=np.uint8)
    rgba = matplotlib.colormaps["RdYlGn"](dexterity.transpose(1, 2, 0))
    colors[..., :3] = (rgba[..., :3] * 255).astype(np.uint8)
    colors[..., 3] = 255

    transform = np.eye(4)
    transform[0, 0] = transform[1, 1] = transform[2, 2] = params.cell
    transform[:3, 3] = (
        -params.r_xy + params.cell / 2,
        -params.r_xy + params.cell / 2,
        params.z_min + params.cell / 2,
    )
    grid = trimesh.voxel.VoxelGrid(matrix, transform=transform)  # type: ignore[no-untyped-call]
    return grid.as_boxes(colors=colors), n_voxels  # type: ignore[no-untyped-call]


def slice_image_yaw(
    cap: CapabilityMap, yaw_deg: float, px_per_cell: int = 6
) -> tuple[np.ndarray, float, float]:
    """Dexterity cross-section along the vertical plane through the pelvis
    axis at the given yaw. Returns (RGB image, width_m, height_m); image x
    spans [-r_xy, r_xy] along the yaw direction, y spans [z_min, z_max]."""
    params = cap.params
    n_s = params.n_xy * px_per_cell
    n_z = params.n_z * px_per_cell
    s = np.linspace(-params.r_xy, params.r_xy, n_s)
    z = np.linspace(params.z_max, params.z_min, n_z)  # row 0 = top
    yaw = np.deg2rad(yaw_deg)
    xs = np.cos(yaw) * s
    ys = np.sin(yaw) * s
    positions = np.stack(
        [
            np.broadcast_to(xs, (n_z, n_s)).reshape(-1),
            np.broadcast_to(ys, (n_z, n_s)).reshape(-1),
            np.broadcast_to(z[:, None], (n_z, n_s)).reshape(-1),
        ],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n_z, n_s))
    return image, 2 * params.r_xy, params.z_max - params.z_min


def slice_image_height(
    cap: CapabilityMap, z: float, px_per_cell: int = 6
) -> tuple[np.ndarray, float, float]:
    """Dexterity cross-section on the horizontal plane at height z."""
    params = cap.params
    n = params.n_xy * px_per_cell
    axis = np.linspace(-params.r_xy, params.r_xy, n)
    xx, yy = np.meshgrid(axis, -axis)  # row 0 = +y edge so the image reads like a map
    positions = np.stack(
        [xx.reshape(-1), yy.reshape(-1), np.full(n * n, z)],
        axis=1,
    )
    image = _dexterity_image(cap, positions, (n, n))
    return image, 2 * params.r_xy, 2 * params.r_xy


def _dexterity_image(
    cap: CapabilityMap, positions: np.ndarray, shape: tuple[int, int]
) -> np.ndarray:
    import matplotlib

    dexterity = cap.body_dexterity()
    iz, ix, iy, valid = cap.body_indices(positions)
    values = np.zeros(len(positions))
    occupied = np.zeros(len(positions), dtype=bool)
    values[valid] = dexterity[iz[valid], ix[valid], iy[valid]]
    occupied[valid] = cap.body_counts[iz[valid], ix[valid], iy[valid]] > 0

    rgba = matplotlib.colormaps["RdYlGn"](np.clip(values / max(values.max(), 1e-9), 0, 1))
    image = (rgba[:, :3] * 255).astype(np.uint8)
    image[~occupied] = _EMPTY_GRAY
    return np.asarray(image.reshape(*shape, 3))


def score_colors(scores: np.ndarray, vmax: float | None = None) -> np.ndarray:
    """Score → red-to-green uint8 colors (red = barely reachable, green = rich)."""
    import matplotlib

    vmax = vmax or max(float(scores.max(initial=1.0)), 1.0)
    rgba = matplotlib.colormaps["RdYlGn"](np.clip(scores / vmax, 0, 1))
    return (rgba[:, :3] * 255).astype(np.uint8)


# ----------------------------------------------------------------------
# Live IK (poses the URDF arm at the gizmo target)


class ArmIK:
    """mink QP IK for one arm on the construction MJCF; non-arm DOF masked.

    Self-collision is handled the same way construction does (any contact
    involving the arm's moving subtree): a CollisionAvoidanceLimit steers
    the QP away from contact, and a post-hoc mj_collision check rejects
    solutions that still penetrate."""

    def __init__(self, side: str) -> None:
        import mink

        from dimos.manipulation.reachability.construct import _ArmSampler, g1_spec

        self._mink = mink
        spec = g1_spec(side)
        self._sampler = _ArmSampler(spec)
        self.joint_names = list(spec.joint_names)
        model = self._sampler.model
        self._configuration = mink.Configuration(model)
        self._frame = mink.FrameTask(
            frame_name=spec.ee_body,
            frame_type="body",
            position_cost=1.0,
            orientation_cost=0.6,
            lm_damping=1.0,
        )
        claimed = {
            int(model.jnt_dofadr[j])
            for j in range(model.njnt)
            if int(model.jnt_qposadr[j]) in set(self._sampler.qpos_adr.tolist())
        }
        self._velocity_mask = np.zeros(model.nv)
        self._velocity_mask[list(claimed)] = 1.0
        self._tasks = [self._frame]
        # Arm-vs-rest and arm-vs-arm avoidance pairs; mink filters welded,
        # parent-child, and contype/conaffinity-incompatible combinations.
        collidable = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
        arm_geoms = [int(g) for g in np.flatnonzero(self._sampler.check_geom_mask & collidable)]
        other_geoms = [int(g) for g in np.flatnonzero(~self._sampler.check_geom_mask & collidable)]
        self._limits = [
            mink.ConfigurationLimit(model),
            mink.CollisionAvoidanceLimit(model, [(arm_geoms, other_geoms), (arm_geoms, arm_geoms)]),
        ]
        self._q_warm = self._sampler._q_base.copy()
        # Persistent across solves: a failed solve retried with the same
        # seed would re-attempt the exact same restart configurations and
        # fail identically forever.
        self._rng = np.random.default_rng(0)

    def solve(
        self,
        position: np.ndarray,
        wxyz: np.ndarray,
        restarts: int = 5,
        on_step: Callable[[dict[str, float], float, int], bool | None] | None = None,
    ) -> tuple[dict[str, float], bool, float, bool]:
        """Solve toward a grasp-center target; returns (arm joints by model
        joint name, reached?, position error in m, self-colliding?).

        Warm-started from the previous solve so dragging feels continuous;
        falls back to random restarts when the warm start stalls in a local
        minimum. Collision-free solutions are preferred over closer ones
        that penetrate; ``reached`` requires both tolerance and no contact.

        ``on_step(joints, error_m, attempt)`` is called every few descent
        iterations with the solver's *current* (possibly wrong) guess, so a
        caller can animate the search instead of blocking silently. If the
        callback returns a truthy value the search aborts early (the target
        is stale — a newer one is waiting) and the best result so far is
        returned."""
        import mujoco

        mink = self._mink
        sampler = self._sampler
        rotation_flat = np.empty(9)
        mujoco.mju_quat2Mat(rotation_flat, np.asarray(wxyz, dtype=np.float64))
        rotation = rotation_flat.reshape(3, 3)
        body_position = np.asarray(position) - rotation @ sampler.grasp_offset
        self._frame.set_target(
            mink.SE3.from_rotation_and_translation(mink.SO3(np.asarray(wxyz)), body_position)
        )

        best_q, best_error, best_free = None, np.inf, False
        aborted = False
        for attempt in range(1 + restarts):
            q0 = self._q_warm.copy()
            if attempt == 1:
                # The home pose: a deterministic, collision-free recovery
                # basin in case the warm start is stuck somewhere bad.
                q0 = sampler._q_base.copy()
            elif attempt > 1:
                q0[sampler.qpos_adr] = self._rng.uniform(sampler.lower, sampler.upper)
            self._configuration.update(q0)
            # The task error decays ~×(1 − gain·dt) per iteration; 300 steps
            # at dt=0.05 reduce it by ~2e-7. Stopping at 60 left ~5% of the
            # initial error (tens of mm) and looked like an IK failure.
            for iteration in range(300):
                velocity = (
                    mink.solve_ik(
                        self._configuration, self._tasks, 0.05, "daqp", limits=self._limits
                    )
                    * self._velocity_mask
                )
                self._configuration.integrate_inplace(velocity, 0.05)
                if on_step is not None and iteration % 12 == 0:
                    q_now = self._configuration.q
                    aborted = bool(
                        on_step(
                            self._joints_of(q_now), self._position_error(q_now, position), attempt
                        )
                    )
                    if aborted:
                        break
                if float(np.linalg.norm(velocity)) < 1e-4:
                    break
            error = self._position_error(self._configuration.q, position)
            free = not self._self_collides(self._configuration.q)
            if (free, -error) > (best_free, -best_error):
                best_q, best_error, best_free = self._configuration.q.copy(), error, free
            if aborted or (best_free and best_error < 0.01):
                break

        assert best_q is not None
        # Only a collision-free result may seed the next warm start: warming
        # from a colliding/stuck configuration poisons every following solve.
        if best_free:
            self._q_warm = best_q.copy()
        return (
            self._joints_of(best_q),
            best_free and best_error < 0.02,
            best_error,
            not best_free,
        )

    def _joints_of(self, q: np.ndarray) -> dict[str, float]:
        return {
            name: float(q[adr])
            for name, adr in zip(self.joint_names, self._sampler.qpos_adr, strict=True)
        }

    def _self_collides(self, q: np.ndarray) -> bool:
        """Construction-identical check: any penetrating contact involving
        the arm's moving subtree."""
        import mujoco

        sampler = self._sampler
        data, model = sampler.data, sampler.model
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        if not data.ncon:
            return False
        geom = data.contact.geom[: data.ncon]
        dist = data.contact.dist[: data.ncon]
        involved = sampler.check_geom_mask[geom[:, 0]] | sampler.check_geom_mask[geom[:, 1]]
        return bool(np.any(involved & (dist < 0.0)))

    def _position_error(self, q: np.ndarray, target_position: np.ndarray) -> float:
        import mujoco

        sampler = self._sampler
        data, model = sampler.data, sampler.model
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        xmat = data.xmat[sampler.ee_body_id].reshape(3, 3)
        reached = data.xpos[sampler.ee_body_id] + xmat @ sampler.grasp_offset
        return float(np.linalg.norm(reached - np.asarray(target_position)))


# ----------------------------------------------------------------------
# Server


def _add_g1(server: Any, pelvis_height: float) -> tuple[Any, list[str]]:
    """G1 URDF at the map pelvis pose; returns (viser_urdf | None, joint names)."""
    try:
        from viser.extras import ViserUrdf
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(_G1_URDF))
        viser_urdf = ViserUrdf(server, urdf, root_node_name="/g1")
        server.scene.add_frame("/g1", position=(0.0, 0.0, pelvis_height), show_axes=False)
        names = list(viser_urdf.get_actuated_joint_names())
        viser_urdf.update_cfg(np.zeros(len(names)))
        return viser_urdf, names
    except Exception as e:  # context only — everything else works without it
        logger.warning(f"G1 URDF context unavailable ({e})")
        server.scene.add_frame(
            "/g1", position=(0.0, 0.0, pelvis_height), show_axes=True, axes_length=0.25
        )
        return None, []


def serve(maps: dict[str, CapabilityMap], port: int = 8082) -> None:
    """Start the one-shot viewer (blocks until Ctrl-C)."""
    import viser

    server = viser.ViserServer(host="0.0.0.0", port=port)
    first = next(iter(maps.values()))
    params = first.params
    server.scene.add_grid("/ground", width=4.0, height=4.0, cell_size=0.25)
    viser_urdf, urdf_joint_names = _add_g1(server, params.pelvis_height)

    with server.gui.add_folder("view"):
        side = server.gui.add_dropdown("arm", tuple(maps), initial_value=next(iter(maps)))
        style = server.gui.add_dropdown("style", ("points", "voxels"), initial_value="points")
        point_size = server.gui.add_slider(
            "point size [mm]", min=0, max=60, step=1, initial_value=5
        )
        dexterity_pct = server.gui.add_slider(
            "min dexterity [%]", min=0, max=60, step=1, initial_value=0
        )

    with server.gui.add_folder("slices"):
        show_yaw_slice = server.gui.add_checkbox("vertical slice", initial_value=False)
        yaw_slice = server.gui.add_slider(
            "slice yaw [deg]", min=-180, max=180, step=5, initial_value=0
        )
        show_z_slice = server.gui.add_checkbox("horizontal slice", initial_value=False)
        z_slice = server.gui.add_slider(
            "slice height [m]",
            min=params.z_min,
            max=params.z_max,
            step=params.cell,
            initial_value=0.9,
        )

    with server.gui.add_folder("IK target"):
        ik_enabled = server.gui.add_checkbox("drag-to-reach", initial_value=False)
        ik_status = server.gui.add_text(
            "status", initial_value="enable to pose the arm", disabled=True
        )

    solvers: dict[str, ArmIK | None] = {}
    gizmo = None

    def current_map() -> CapabilityMap:
        return maps[side.value]

    def refresh_volume(_: Any = None) -> None:
        cap = current_map()
        for name in ("/reachability/core", "/reachability/points"):
            try:
                server.scene.remove_by_name(name)
            except Exception:
                pass
        n = 0
        if style.value == "voxels":
            core, n = body_voxel_mesh(cap, dexterity_pct.value / 100.0)
            if core is not None:
                server.scene.add_mesh_trimesh("/reachability/core", core)
        else:
            points, dexterity = body_point_cloud(cap, dexterity_pct.value / 100.0)
            n = len(points)
            if n and point_size.value > 0:
                server.scene.add_point_cloud(
                    "/reachability/points",
                    points=points.astype(np.float32),
                    colors=score_colors(dexterity, vmax=max(float(dexterity.max()), 1e-9)),
                    point_size=point_size.value / 1000.0,
                    point_shape="circle",
                )
        logger.info(f"workspace view: {n} cells at ≥{dexterity_pct.value}% dexterity")

    def refresh_slices(_: Any = None) -> None:
        cap = current_map()
        try:
            server.scene.remove_by_name("/slice/yaw")
        except Exception:
            pass
        try:
            server.scene.remove_by_name("/slice/z")
        except Exception:
            pass
        # viser's add_image uses the camera convention: image rows run along
        # the node's local +y (row 0 at -y). The slice images are standard
        # row-0-on-top, so flip rows to land top-of-image at +local-y.
        if show_yaw_slice.value:
            image, width, height = slice_image_yaw(cap, yaw_slice.value)
            yaw = np.deg2rad(yaw_slice.value)
            server.scene.add_image(
                "/slice/yaw",
                np.ascontiguousarray(image[::-1]),
                render_width=width,
                render_height=height,
                position=(0.0, 0.0, (params.z_min + params.z_max) / 2),
                wxyz=_plane_wxyz(yaw),
            )
        if show_z_slice.value:
            image, width, height = slice_image_height(cap, z_slice.value)
            server.scene.add_image(
                "/slice/z",
                np.ascontiguousarray(image[::-1]),
                render_width=width,
                render_height=height,
                position=(0.0, 0.0, float(z_slice.value)),
                wxyz=(1.0, 0.0, 0.0, 0.0),
            )

    # IK runs on its own worker so drag events never queue behind a slow
    # search: each event just pokes the worker, which always solves for the
    # gizmo's *current* pose (latest wins), aborts a search mid-descent
    # when the target moved, and retries a failed solve on its own.
    ik_wakeup = threading.Event()

    def refresh_ik(_: Any = None) -> None:
        nonlocal gizmo
        if not ik_enabled.value:
            if gizmo is not None:
                gizmo.remove()
                gizmo = None
            return
        if gizmo is None:
            gizmo = server.scene.add_transform_controls(
                "/ik_target", scale=0.18, position=(0.35, 0.2, 1.05)
            )
            gizmo.on_update(lambda _: ik_wakeup.set())
        ik_wakeup.set()

    def pose_ghost(joints: dict[str, float]) -> None:
        if viser_urdf is None or not urdf_joint_names:
            return
        cfg = np.zeros(len(urdf_joint_names))
        for i, name in enumerate(urdf_joint_names):
            if name in joints:
                cfg[i] = joints[name]
        viser_urdf.update_cfg(cfg)

    def solve_current_pose() -> bool:
        """One solve at the gizmo's current pose; False if it failed."""
        if gizmo is None:
            return True
        cap = current_map()
        if cap.side not in solvers:
            try:
                solvers[cap.side] = ArmIK(cap.side)
            except Exception as e:
                solvers[cap.side] = None
                logger.warning(f"IK unavailable: {e}")
        solver = solvers[cap.side]
        if solver is None:
            ik_status.value = "IK unavailable (pip install 'dimos[ik]')"
            return True
        position = np.asarray(gizmo.position, dtype=np.float64)
        wxyz = np.asarray(gizmo.wxyz, dtype=np.float64)

        def show_guess(joints: dict[str, float], error: float, attempt: int) -> bool:
            # Stream the solver's current (possibly wrong) guess so the
            # search is visible instead of a silent pause; abort when the
            # gizmo has moved on (a fresh wakeup is pending).
            pose_ghost(joints)
            ik_status.value = f"solving... attempt {attempt + 1} | err {error * 1000:.0f} mm"
            return ik_wakeup.is_set()

        joints, reached, error, collided = solver.solve(position, wxyz, on_step=show_guess)
        if ik_wakeup.is_set():
            return True  # stale result; the worker re-solves immediately

        import mujoco

        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, wxyz)
        score = cap.scores(position[None], rotation.reshape(1, 3, 3))[0]
        iz, ix, iy, valid = cap.body_indices(position[None])
        dexterity = cap.body_dexterity()[iz[0], ix[0], iy[0]] if valid[0] else 0.0
        # Rigid-model kinematics: the real G1 arms are compliant and sag a
        # few cm under gravity at low PD gains — treat the posed arm as the
        # commanded pose, not where the hardware would settle.
        verdict = "reached" if reached else "FAILED"
        if collided:
            verdict += ", SELF-COLLISION"
        ik_status.value = (
            f"IK {verdict} (err {error * 1000:.0f} mm) | "
            f"map score {score} | dexterity {dexterity:.0%} | rigid model (no sag)"
        )
        pose_ghost(joints)
        return reached

    def ik_worker() -> None:
        while True:
            ik_wakeup.wait()
            ik_wakeup.clear()
            try:
                solved = solve_current_pose()
                # A failed solve retries with fresh random restarts (the
                # solver's RNG advances) unless a new drag superseded it.
                for _ in range(2):
                    if solved or ik_wakeup.is_set() or gizmo is None:
                        break
                    solved = solve_current_pose()
            except Exception as e:
                logger.warning(f"IK solve failed: {e}")

    threading.Thread(target=ik_worker, daemon=True, name="ik-worker").start()

    for control in (side, style, point_size, dexterity_pct):
        control.on_update(refresh_volume)
    for control in (side, show_yaw_slice, yaw_slice, show_z_slice, z_slice):
        control.on_update(refresh_slices)
    ik_enabled.on_update(refresh_ik)
    side.on_update(lambda _: ik_wakeup.set())

    refresh_volume()
    refresh_slices()
    logger.info(f"Reachability viewer: http://localhost:{port}")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def _plane_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion placing an image plane vertically: local x → yaw direction
    in the ground plane, local y → world +z."""
    from scipy.spatial.transform import Rotation

    matrix = np.array(
        [
            [np.cos(yaw), 0.0, np.sin(yaw)],
            [np.sin(yaw), 0.0, -np.cos(yaw)],
            [0.0, 1.0, 0.0],
        ]
    )
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return (w, x, y, z)


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="Interactive capability-map viewer (viser).")
    parser.add_argument(
        "--map", type=Path, action="append", required=True, help="capability .npz (repeatable)"
    )
    parser.add_argument("--port", type=int, default=8082)
    args = parser.parse_args()

    maps = {}
    for path in args.map:
        cap = CapabilityMap.load(path)
        maps[f"{cap.side} ({path.name})"] = cap
    serve(maps, port=args.port)


if __name__ == "__main__":
    cli_main()


__all__ = [
    "ArmIK",
    "body_point_cloud",
    "body_voxel_mesh",
    "score_colors",
    "serve",
    "slice_image_height",
    "slice_image_yaw",
]
