// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells};
use crate::dijkstra::{dijkstra, dijkstra_region, DijkstraState, Weight};
use crate::voxel::{surface_point_xyz, VoxelKey};

#[derive(Clone, Copy, Debug)]
pub struct NodeData {
    pub cell_id: CellId,
    pub pos: (f32, f32, f32),
}

/// Distribute nodes on the surfaces.
///
/// Runs multi source dijkstra using edges as sources, then distribute nodes
/// using a grid based NMS.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes(
    cells: &mut SurfaceCells,
    voxel_size: f32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
    robot_radius_m: f32,
    wall_penalty_weight: f32,
    state: &mut DijkstraState,
    out_nodes: &mut Vec<NodeData>,
) {
    out_nodes.clear();
    if cells.is_empty() {
        return;
    }

    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_cells(cells, &mut wall_seeds);
    dijkstra(cells, &wall_seeds, state, Weight::Base);

    let node_floor = node_wall_buffer_m.max(robot_radius_m);
    let candidates: Vec<CellId> = cells
        .ids()
        .filter(|&id| state.dist[id as usize] >= node_floor)
        .collect();
    place_from_candidates(
        cells,
        candidates,
        &state.dist,
        &[],
        voxel_size,
        node_spacing_m,
        out_nodes,
    );

    apply_wall_safe_penalty(
        cells,
        &state.dist,
        node_wall_buffer_m,
        robot_radius_m,
        wall_penalty_weight,
    );
}

/// Sort candidates by descending wall distance, thin them with NMS against the
/// seed nodes, and append the survivors as nodes.
fn place_from_candidates(
    cells: &SurfaceCells,
    mut candidates: Vec<CellId>,
    dist: &[f32],
    seeds: &[CellId],
    voxel_size: f32,
    node_spacing_m: f32,
    out_nodes: &mut Vec<NodeData>,
) {
    candidates.par_sort_unstable_by(|&a, &b| {
        dist[b as usize]
            .total_cmp(&dist[a as usize])
            .then(cells.coord(a).cmp(&cells.coord(b)))
    });
    let survivors = nms_grid(cells, &candidates, seeds, voxel_size, node_spacing_m);
    out_nodes.reserve(survivors.len());
    for &id in &survivors {
        let (ix, iy, iz) = cells.coord(id);
        out_nodes.push(NodeData {
            cell_id: id,
            pos: surface_point_xyz(ix, iy, iz, voxel_size),
        });
    }
}

/// Regional counterpart to place_nodes: replace nodes inside the window,
/// keeping cached outside nodes as NMS seeds.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes_region(
    cells: &mut SurfaceCells,
    window: &AHashSet<CellId>,
    voxel_size: f32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
    robot_radius_m: f32,
    wall_penalty_weight: f32,
    wall_state: &mut DijkstraState,
    nodes: &mut Vec<NodeData>,
) {
    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_in_window(cells, window, &mut wall_seeds);
    dijkstra_region(cells, &wall_seeds, window, wall_state, Weight::Base);

    nodes.retain(|n| cells.is_live(n.cell_id) && !window.contains(&n.cell_id));
    let kept: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();

    let node_floor = node_wall_buffer_m.max(robot_radius_m);
    let candidates: Vec<CellId> = window
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id) && wall_state.dist[id as usize] >= node_floor)
        .collect();
    place_from_candidates(
        cells,
        candidates,
        &wall_state.dist,
        &kept,
        voxel_size,
        node_spacing_m,
        nodes,
    );

    apply_wall_safe_penalty_region(
        cells,
        &wall_state.dist,
        node_wall_buffer_m,
        robot_radius_m,
        wall_penalty_weight,
        window,
    );
}

/// Wall-adjacency over a cell subset, matching collect_wall_adjacent_cells.
fn collect_wall_adjacent_in_window(
    cells: &SurfaceCells,
    window: &AHashSet<CellId>,
    out: &mut Vec<CellId>,
) {
    out.clear();
    for &id in window {
        if cells.is_live(id) && is_wall_adjacent(cells, id) {
            out.push(id);
        }
    }
}

/// A cell is wall-adjacent when it is missing at least one of its 4 xy-direction
/// neighbors. Membership is tracked with a 4-bit mask to avoid per-cell
/// allocation on the 349k-cell case.
fn is_wall_adjacent(cells: &SurfaceCells, id: CellId) -> bool {
    let (cx, cy, _) = cells.coord(id);
    let mut mask: u8 = 0;
    for e in cells.neighbors(id) {
        let (nx, ny, _) = cells.coord(e.dest);
        mask |= match (nx - cx, ny - cy) {
            (-1, 0) => 1,
            (1, 0) => 2,
            (0, -1) => 4,
            (0, 1) => 8,
            _ => 0,
        };
    }
    mask != 0b1111
}

/// Rescale edge costs for the window and its neighbors, whose wall distance may
/// have changed. Idempotent via base_cost.
fn apply_wall_safe_penalty_region(
    cells: &mut SurfaceCells,
    dist: &[f32],
    buffer_m: f32,
    robot_radius_m: f32,
    weight: f32,
    window: &AHashSet<CellId>,
) {
    let mut affected: AHashSet<CellId> = AHashSet::with_capacity(window.len() * 2);
    for &w in window {
        affected.insert(w);
        for e in cells.neighbors(w) {
            affected.insert(e.dest);
        }
    }
    for id in affected {
        scale_edges(
            cells.edges_mut(id),
            id,
            dist,
            buffer_m,
            robot_radius_m,
            weight,
        );
    }
}

/// Wall-adjacent cells over the whole graph. Falls back to a single cell so a
/// fully-enclosed map still seeds the wall-distance field.
fn collect_wall_adjacent_cells(cells: &SurfaceCells, out: &mut Vec<CellId>) {
    out.clear();
    for id in cells.ids() {
        if is_wall_adjacent(cells, id) {
            out.push(id);
        }
    }
    if out.is_empty() {
        if let Some(c) = cells.ids().next() {
            out.push(c);
        }
    }
}

/// Space out nodes based on minimum distance.
///
/// The seed nodes suppress nearby candidates without being emitted, keeping a
/// regional re-placement consistent with cached nodes outside the window.
fn nms_grid(
    cells: &SurfaceCells,
    candidates_sorted: &[CellId],
    seeds: &[CellId],
    voxel_size: f32,
    node_spacing_m: f32,
) -> Vec<CellId> {
    let bin_size = ((node_spacing_m / voxel_size) as i32).max(1);
    let r_sq = (node_spacing_m as f64) * (node_spacing_m as f64);
    let v = voxel_size as f64;
    let bin_of = |c: VoxelKey| {
        (
            c.0.div_euclid(bin_size),
            c.1.div_euclid(bin_size),
            c.2.div_euclid(bin_size),
        )
    };

    let mut bins: AHashMap<(i32, i32, i32), Vec<CellId>> = AHashMap::new();
    for &s in seeds {
        bins.entry(bin_of(cells.coord(s))).or_default().push(s);
    }
    let mut survivors: Vec<CellId> = Vec::new();
    for &id in candidates_sorted {
        let coord = cells.coord(id);
        let (bx, by, bz) = bin_of(coord);
        let mut killed = false;
        'outer: for dbx in -1..=1 {
            for dby in -1..=1 {
                for dbz in -1..=1 {
                    if let Some(nearby) = bins.get(&(bx + dbx, by + dby, bz + dbz)) {
                        for &n_id in nearby {
                            let n = cells.coord(n_id);
                            let dx = (coord.0 - n.0) as f64 * v;
                            let dy = (coord.1 - n.1) as f64 * v;
                            let dz = (coord.2 - n.2) as f64 * v;
                            if dx * dx + dy * dy + dz * dz <= r_sq {
                                killed = true;
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        if !killed {
            survivors.push(id);
            bins.entry((bx, by, bz)).or_default().push(id);
        }
    }
    survivors
}

/// Scale every edge cost by the average of its endpoint penalties, which
/// pushes shortest paths away from walls and forbids sub-radius cells.
/// Unreached cells have dist == +INFINITY which collapses to penalty 1.0.
fn apply_wall_safe_penalty(
    cells: &mut SurfaceCells,
    dist: &[f32],
    buffer_m: f32,
    robot_radius_m: f32,
    weight: f32,
) {
    let mut edge_lists: Vec<(CellId, &mut Vec<Edge>)> = cells.iter_edges_mut().collect();
    edge_lists.par_iter_mut().for_each(|(src, edges)| {
        scale_edges(edges, *src, dist, buffer_m, robot_radius_m, weight);
    });
}

/// Rescale one cell's outgoing edges from base_cost. Idempotent, so a regional
/// repass cannot compound the penalty.
#[inline]
fn scale_edges(
    edges: &mut [Edge],
    src: CellId,
    dist: &[f32],
    buffer_m: f32,
    robot_radius_m: f32,
    weight: f32,
) {
    let pu = penalty_of(dist[src as usize], buffer_m, robot_radius_m, weight);
    for edge in edges.iter_mut() {
        let pv = penalty_of(dist[edge.dest as usize], buffer_m, robot_radius_m, weight);
        edge.cost = edge.base_cost * (pu + pv) / 2.0;
    }
}

/// Cost multiplier at wall distance d. Infinite inside the robot radius,
/// then decays from 1 + weight toward 1 with length scale buffer_m.
#[inline]
fn penalty_of(d: f32, buffer_m: f32, robot_radius_m: f32, weight: f32) -> f32 {
    if d < robot_radius_m {
        return f32::INFINITY;
    }
    let scale = buffer_m.max(1e-3);
    1.0 + weight * (-(d - robot_radius_m) / scale).exp()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup, SurfaceLookup};

    const VOXEL: f32 = 0.1;

    fn open_patch(ix0: i32, iy0: i32, size: i32) -> Vec<VoxelKey> {
        let mut c = Vec::new();
        for dx in 0..size {
            for dy in 0..size {
                c.push((ix0 + dx, iy0 + dy, 0));
            }
        }
        c
    }

    fn build_cells(surface: &[VoxelKey], step_cells: i32) -> SurfaceCells {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(surface, &mut lookup);
        let mut sc = SurfaceCells::default();
        build_surface_cells(&mut sc, &lookup, VOXEL, step_cells);
        sc
    }

    #[test]
    fn open_patch_places_at_least_one_node() {
        let mut sc = build_cells(&open_patch(0, 0, 10), 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, 0.0, 1.0, &mut state, &mut nodes);
        assert!(!nodes.is_empty());
        for n in &nodes {
            let (ix, iy, _) = sc.coord(n.cell_id);
            assert!((0..10).contains(&ix) && (0..10).contains(&iy));
        }
    }

    #[test]
    fn sloped_patch_places_interior_nodes() {
        let mut cells_in = Vec::new();
        for ix in 0..10 {
            for iy in 0..10 {
                cells_in.push((ix, iy, ix));
            }
        }
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, 0.0, 1.0, &mut state, &mut nodes);
        assert!(!nodes.is_empty());
    }

    #[test]
    fn nms_enforces_spacing() {
        let mut cells_in = open_patch(0, 0, 10);
        cells_in.extend(open_patch(20, 0, 10));
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, 0.0, 1.0, &mut state, &mut nodes);
        assert!(nodes.len() >= 2);
        for i in 0..nodes.len() {
            for j in (i + 1)..nodes.len() {
                let a = nodes[i].pos;
                let b = nodes[j].pos;
                let dx = a.0 - b.0;
                let dy = a.1 - b.1;
                let dz = a.2 - b.2;
                let d_sq = dx * dx + dy * dy + dz * dz;
                assert!(d_sq > 1.0 * 1.0 - 1e-4);
            }
        }
    }

    #[test]
    fn wall_penalty_weight_scales_edge_costs() {
        // On a 1-wide strip every cell is wall-adjacent, so the penalty
        // multiplier is exactly 1 + weight and edge cost is base times it.
        let cells_in: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let cost_with = |weight: f32| {
            let mut sc = build_cells(&cells_in, 2);
            let mut state = DijkstraState::default();
            let mut nodes = Vec::new();
            place_nodes(
                &mut sc, VOXEL, 1.0, 0.3, 0.0, weight, &mut state, &mut nodes,
            );
            let id = sc.id((5, 0, 0)).unwrap();
            sc.neighbors(id)[0].cost
        };
        let unweighted = cost_with(0.0);
        assert!(
            (unweighted - VOXEL).abs() < 1e-5,
            "zero weight must leave the geometric cost, got {unweighted}"
        );
        assert!(
            (cost_with(4.0) - 5.0 * VOXEL).abs() < 1e-5,
            "weight 4 at the wall must scale cost by 5"
        );
        assert!(cost_with(4.0) > cost_with(1.0));
    }

    #[test]
    fn wall_cells_scale_outbound_cost() {
        let cells_in: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, 0.0, 1.0, &mut state, &mut nodes);
        let id0 = sc.id((0, 0, 0)).unwrap();
        let outbound = sc.neighbors(id0);
        assert!(!outbound.is_empty());
        for edge in outbound {
            assert!(edge.cost >= 1.5 * VOXEL - 1e-5);
        }
    }
}
