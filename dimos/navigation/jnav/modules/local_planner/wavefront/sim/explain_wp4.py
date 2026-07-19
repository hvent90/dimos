# Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
"""Top-down before/after diagram of the wp4 stall and the route-following fix."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dimos.navigation.jnav.modules.local_planner.wavefront import local_planner as L

RES = 0.1
ORIGIN = (0.0, 0.0)
OUT = Path("/home/dimos/.local/share/cbg/long-tasks/BigFish/proof/wp4_explained.png")

# --- scene: wp4-like. Goal is inside a room that is sealed on every side present
# in THIS local costmap; its real door is a ~20 m loop away, off-map to the west.
W, H = 110, 80  # 11 x 8 m
cost = np.zeros((H, W))


def wall(x0, x1, y0, y1):
    cost[int(y0 / RES) : int(y1 / RES), int(x0 / RES) : int(x1 / RES)] = 100


# Sealed room around the goal (a disconnected island in this window).
wall(5.0, 8.0, 5.5, 5.7)  # south wall (faces the robot)
wall(5.0, 8.0, 7.5, 7.7)  # north wall
wall(5.0, 5.2, 5.5, 7.7)  # west wall
wall(7.8, 8.0, 5.5, 7.7)  # east wall

robot = (6.5, 2.0, 0.0)
goal = (6.5, 6.6)  # inside the sealed room
# MLS route: head west along the open corridor toward the (off-map) door, then
# loop up and back into the room. Only the westward leg is inside this window.
route = [(6.5, 2.0), (3.5, 2.0), (0.3, 2.0), goal]

params = L.WavefrontParams(vehicle_width=0.5)

# --- NEW planner output (current code: follow the route) ---------------------
new_poses = L.plan_path(cost, RES, ORIGIN, route, robot, params)
new_xy = np.array([(x, y) for x, y, _ in new_poses])

# --- OLD behaviour (target = reachable cell closest to the goal) --------------
obstacle, dist = L._obstacle_distance(cost, RES, params)
rr = params.vehicle_width * 0.5
free = ~obstacle & (dist >= rr)
robot_cell = L._nearest_free_cell(free, L._world_to_cell(*robot[:2], RES, ORIGIN, (H, W)))
entry = (
    1.0
    + params.clearance_weight * L._clearance_penalty(dist, rr, params.influence_radius)
    + params.path_weight * L._path_distance(np.asarray(route, float), (H, W), RES, ORIGIN)
)
_, parent = L._dijkstra_tree(free, entry, RES, robot_cell)
reach = parent[:, :, 0] >= 0
reach[robot_cell] = True
gr, gc = L._world_to_cell(*goal, RES, ORIGIN, (H, W))
rows, cols = np.arange(H)[:, None], np.arange(W)[None, :]
d2 = np.where(reach, (rows - gr) ** 2 + (cols - gc) ** 2, np.inf)
flat = int(np.argmin(d2))
old_cells = L._backtrack(parent, robot_cell, (flat // W, flat % W))
old_xy = np.array([L._cell_center_world(r, c, RES, ORIGIN) for r, c in old_cells])

# --- plot --------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(15, 6))
extent = [0, W * RES, 0, H * RES]
short = float(np.hypot(old_xy[-1, 0] - goal[0], old_xy[-1, 1] - goal[1]))


def draw_base(ax, title):
    ax.imshow(
        np.where(cost >= 50, 1.0, 0.0), origin="lower", extent=extent, cmap="Reds", alpha=0.55, vmax=1.5
    )
    ax.plot(*robot[:2], "o", color="#2ca02c", ms=13, label="robot", zorder=5)
    ax.plot(*goal, "*", color="#d62728", ms=20, label="wp4 goal (in sealed room)", zorder=5)
    # the route MLS gives us (dashed), and the off-map door it leads to
    rx = [p[0] for p in route[:-1]]
    ry = [p[1] for p in route[:-1]]
    ax.plot(rx, ry, "--", color="#888", lw=2, label="MLS global route")
    ax.annotate(
        "to door:\n~20 m loop,\noff-map →",
        xy=(0.1, 2.0), xytext=(0.2, 3.6), color="#555", fontsize=9,
        arrowprops=dict(arrowstyle="->", color="#555"),
    )
    ax.text(6.5, 6.6, "  room sealed\n  in this costmap", fontsize=8, color="#a00", va="center")
    ax.set_xlim(0, W * RES)
    ax.set_ylim(0, H * RES)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=13)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


draw_base(axes[0], f"BEFORE — aim at the goal → stall {short:.2f} m short")
axes[0].plot(old_xy[:, 0], old_xy[:, 1], "-", color="#d62728", lw=3, label="local path (old)")
axes[0].plot(old_xy[-1, 0], old_xy[-1, 1], "X", color="#d62728", ms=14, mew=3)
axes[0].annotate(
    "✗ stops dead at the wall\n(closest cell to the goal)",
    xy=(old_xy[-1, 0], old_xy[-1, 1]), xytext=(7.0, 3.6), color="#d62728", fontsize=10,
    arrowprops=dict(arrowstyle="->", color="#d62728"),
)
axes[0].legend(loc="upper left", fontsize=8)

draw_base(axes[1], "AFTER — follow the route toward the door")
axes[1].plot(new_xy[:, 0], new_xy[:, 1], "-", color="#1f77b4", lw=3, label="local path (new)")
axes[1].plot(new_xy[-1, 0], new_xy[-1, 1], "X", color="#1f77b4", ms=14, mew=3)
axes[1].annotate(
    "✓ drives along the route\ntoward the door, then\nthrough it as the costmap\nscrolls to include it",
    xy=(new_xy[-1, 0], new_xy[-1, 1]), xytext=(2.2, 4.2), color="#1f77b4", fontsize=10,
    arrowprops=dict(arrowstyle="->", color="#1f77b4"),
)
axes[1].legend(loc="upper left", fontsize=8)

fig.suptitle(
    "wp4 stall: the goal's room is a disconnected island in the local costmap "
    "(its door is a long loop away, off-map)",
    fontsize=13,
)
fig.tight_layout(rect=(0, 0, 1, 0.96))
OUT.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT, dpi=110)
print("saved", OUT, "| old stops", round(short, 2), "m short | new ends x=", round(new_xy[-1, 0], 2))
