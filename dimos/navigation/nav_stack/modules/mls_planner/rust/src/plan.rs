// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Query-time planning: snap start/goal poses to nodes, run shortest path on
//! the node graph, and stitch cached edge cell paths into XYZ waypoints.
#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::edges::{walk_preds_to_source, PlannerGraph};
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Snap a query pose to the nearest node by 3D Euclidean distance, rejecting
/// nodes whose z differs from the pose's z by more than `z_tolerance_m`.
pub fn snap_pose_to_node(
    plg: &PlannerGraph,
    pose: (f32, f32, f32),
    z_tolerance_m: f32,
) -> Option<u32> {
    let mut best: Option<(f32, u32)> = None;
    for (i, n) in plg.nodes.iter().enumerate() {
        if (pose.2 - n.pos.2).abs() > z_tolerance_m {
            continue;
        }
        let dx = pose.0 - n.pos.0;
        let dy = pose.1 - n.pos.1;
        let dz = pose.2 - n.pos.2;
        let d_sq = dx * dx + dy * dy + dz * dz;
        match best {
            Some((b, _)) if b <= d_sq => {}
            _ => best = Some((d_sq, i as u32)),
        }
    }
    best.map(|(_, i)| i)
}

/// Plan an XYZ waypoint sequence from `start_pose` to `goal_pose`.
/// Returns None if either pose can't snap, or if the snapped nodes are
/// disconnected in the node graph.
pub fn plan(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
    z_tolerance_m: f32,
) -> Option<Vec<(f32, f32, f32)>> {
    let start_node = snap_pose_to_node(plg, start_pose, z_tolerance_m)?;
    let goal_node = snap_pose_to_node(plg, goal_pose, z_tolerance_m)?;
    let node_seq = shortest_path_nodes(plg, start_node, goal_node)?;
    Some(assemble_waypoints(
        plg, &node_seq, start_pose, goal_pose, voxel_size,
    ))
}

pub fn shortest_path_nodes(plg: &PlannerGraph, start: u32, goal: u32) -> Option<Vec<u32>> {
    if start == goal {
        return Some(vec![start]);
    }
    let n = plg.nodes.len();
    let mut dist = vec![f32::INFINITY; n];
    let mut pred = vec![-1i32; n];
    dist[start as usize] = 0.0;
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    heap.push(Scored(0.0, start));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist[u as usize] {
            continue;
        }
        if u == goal {
            break;
        }
        for &edge_idx in &plg.node_adj[u as usize] {
            let edge = &plg.node_edges[edge_idx as usize];
            let neighbor = if edge.a == u { edge.b } else { edge.a };
            let nd = d + edge.cost;
            if nd < dist[neighbor as usize] {
                dist[neighbor as usize] = nd;
                pred[neighbor as usize] = u as i32;
                heap.push(Scored(nd, neighbor));
            }
        }
    }

    if !dist[goal as usize].is_finite() {
        return None;
    }
    let mut path = vec![goal];
    let mut cur = goal as i32;
    while pred[cur as usize] >= 0 {
        cur = pred[cur as usize];
        path.push(cur as u32);
    }
    path.reverse();
    Some(path)
}

fn assemble_waypoints(
    plg: &PlannerGraph,
    node_seq: &[u32],
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
) -> Vec<(f32, f32, f32)> {
    let mut cells: Vec<VoxelKey> = Vec::new();
    for pair in node_seq.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let edge_idx =
            edge_between(plg, a, b).expect("consecutive nodes in path must share an edge");
        let edge = &plg.node_edges[edge_idx as usize];
        let (start_side, end_side) = if a == edge.a {
            (edge.boundary_u, edge.boundary_v)
        } else {
            (edge.boundary_v, edge.boundary_u)
        };

        let mut from_a = walk_preds_to_source(plg, start_side);
        from_a.reverse();
        let to_b = walk_preds_to_source(plg, end_side);

        for c in from_a.into_iter().chain(to_b) {
            if cells.last() != Some(&c) {
                cells.push(c);
            }
        }
    }

    let mut waypoints: Vec<(f32, f32, f32)> = Vec::with_capacity(cells.len() + 2);
    waypoints.push(start_pose);
    for (ix, iy, iz) in cells {
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    waypoints.push(goal_pose);
    waypoints
}

fn edge_between(plg: &PlannerGraph, a: u32, b: u32) -> Option<u32> {
    for &edge_idx in &plg.node_adj[a as usize] {
        let edge = &plg.node_edges[edge_idx as usize];
        let other = if edge.a == a { edge.b } else { edge.a };
        if other == b {
            return Some(edge_idx);
        }
    }
    None
}

struct Scored(f32, u32);

impl PartialEq for Scored {
    fn eq(&self, other: &Self) -> bool {
        self.0.total_cmp(&other.0) == Ordering::Equal && self.1 == other.1
    }
}
impl Eq for Scored {}
impl PartialOrd for Scored {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Scored {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_adjacency, build_surface_lookup};
    use crate::edges::add_node_edges;
    use crate::nodes::{NodeData, SurfaceGraph};

    const VOXEL: f32 = 0.1;
    const Z_TOL: f32 = 1.5;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let surface_lookup = build_surface_lookup(surface_cells);
        let adj = build_surface_adjacency(&surface_lookup, VOXEL, 2);
        let nodes: Vec<NodeData> = node_cells
            .iter()
            .map(|&c| NodeData {
                cell: c,
                pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
            })
            .collect();
        let sg = SurfaceGraph {
            adj,
            surface_lookup,
            nodes,
        };
        add_node_edges(sg)
    }

    fn strip(n: i32) -> Vec<VoxelKey> {
        (0..n).map(|x| (x, 0, 0)).collect()
    }

    #[test]
    fn snap_returns_none_when_no_nodes() {
        let plg = graph_with_nodes(&strip(20), &[]);
        assert!(snap_pose_to_node(&plg, (0.5, 0.0, 0.05), Z_TOL).is_none());
    }

    #[test]
    fn snap_returns_nearest_node() {
        // Nodes at x=3 and x=15 (XYZ positions 0.35 and 1.55).
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let snapped = snap_pose_to_node(&plg, (0.5, 0.0, 0.1), Z_TOL).unwrap();
        assert_eq!(snapped, 0);
        let snapped = snap_pose_to_node(&plg, (1.4, 0.0, 0.1), Z_TOL).unwrap();
        assert_eq!(snapped, 1);
    }

    #[test]
    fn snap_rejects_outside_z_tolerance() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0)]);
        // Node's pos.z = (0+1)*0.1 = 0.1. Pose at z=2.0, tolerance=1.5 → 1.9 > 1.5 → reject.
        assert!(snap_pose_to_node(&plg, (0.5, 0.0, 2.0), 1.5).is_none());
    }

    #[test]
    fn plan_returns_none_if_start_cant_snap() {
        let plg = graph_with_nodes(&strip(20), &[(10, 0, 0)]);
        let result = plan(&plg, (0.5, 0.0, 10.0), (1.0, 0.0, 0.1), VOXEL, Z_TOL);
        assert!(result.is_none());
    }

    #[test]
    fn plan_returns_none_if_disconnected() {
        // Two strips with a gap.
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let plg = graph_with_nodes(&cells, &[(2, 0, 0), (12, 0, 0)]);
        let result = plan(&plg, (0.25, 0.0, 0.1), (1.25, 0.0, 0.1), VOXEL, Z_TOL);
        assert!(result.is_none());
    }

    #[test]
    fn plan_same_start_and_goal_returns_two_waypoints() {
        let plg = graph_with_nodes(&strip(20), &[(10, 0, 0)]);
        let wp = plan(&plg, (1.0, 0.0, 0.05), (1.0, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        assert_eq!(wp.len(), 2);
        assert_eq!(wp[0], (1.0, 0.0, 0.05));
        assert_eq!(wp[1], (1.0, 0.0, 0.05));
    }

    #[test]
    fn plan_produces_monotonic_xy_along_strip() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan(&plg, (0.2, 0.0, 0.05), (1.7, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        // First waypoint is start_pose, last is goal_pose.
        assert_eq!(wp.first(), Some(&(0.2, 0.0, 0.05)));
        assert_eq!(wp.last(), Some(&(1.7, 0.0, 0.05)));
        // Interior waypoints monotonically increase in x.
        for w in wp.windows(2).skip(1).take(wp.len() - 3) {
            assert!(w[1].0 >= w[0].0 - 1e-5);
        }
    }

    #[test]
    fn plan_three_nodes_visits_them_all() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let wp = plan(&plg, (0.2, 0.0, 0.05), (1.9, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        // Each node's XY position should appear in the waypoints.
        let node_xy: Vec<(f32, f32)> = plg.nodes.iter().map(|n| (n.pos.0, n.pos.1)).collect();
        for &(nx, ny) in &node_xy {
            assert!(
                wp.iter()
                    .any(|w| (w.0 - nx).abs() < 1e-5 && (w.1 - ny).abs() < 1e-5),
                "node ({nx}, {ny}) should appear among waypoints"
            );
        }
    }
}
