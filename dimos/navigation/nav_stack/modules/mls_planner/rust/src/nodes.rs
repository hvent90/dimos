// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

#![allow(dead_code)] // consumed incrementally

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::AHashMap;

use crate::adjacency::{
    build_surface_adjacency, build_surface_lookup, CsrAdjacency, SurfaceAdjacency, SurfaceLookup,
};
use crate::voxel::{surface_point_xyz, VoxelKey};

pub struct NodeData {
    pub cell: VoxelKey,
    pub pos: (f32, f32, f32),
}

pub struct SurfaceGraph {
    pub adj: CsrAdjacency,
    pub idx_to_cell: Vec<VoxelKey>,
    pub cell_to_idx: AHashMap<VoxelKey, u32>,
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
}

pub fn place_nodes(
    surface_cells: &[VoxelKey],
    voxel_size: f32,
    step_cells: i32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
) -> SurfaceGraph {
    let surface_lookup = build_surface_lookup(surface_cells);
    let SurfaceAdjacency {
        adj,
        idx_to_cell,
        cell_to_idx,
    } = build_surface_adjacency(&surface_lookup, voxel_size, step_cells);
    let n = adj.n as usize;

    if n == 0 {
        return SurfaceGraph {
            adj,
            idx_to_cell,
            cell_to_idx,
            surface_lookup,
            nodes: Vec::new(),
        };
    }

    let wall_seeds = wall_adjacent_cells(&adj, &idx_to_cell);
    let dist = dijkstra_multi_source(&adj, &wall_seeds);

    let mut candidates: Vec<u32> = (0..n as u32)
        .filter(|&i| {
            let d = dist[i as usize];
            d.is_finite() && d >= node_wall_buffer_m
        })
        .collect();
    candidates.sort_by(|&a, &b| dist[b as usize].total_cmp(&dist[a as usize]));

    let survivors = nms_grid(&candidates, &idx_to_cell, voxel_size, node_spacing_m);

    let nodes: Vec<NodeData> = survivors
        .iter()
        .map(|&idx| {
            let cell = idx_to_cell[idx as usize];
            NodeData {
                cell,
                pos: surface_point_xyz(cell.0, cell.1, cell.2, voxel_size),
            }
        })
        .collect();

    let scaled_adj = wall_safe_adjacency(&adj, &dist, node_wall_buffer_m);

    SurfaceGraph {
        adj: scaled_adj,
        idx_to_cell,
        cell_to_idx,
        surface_lookup,
        nodes,
    }
}

/// Cells with fewer than 4 same-z neighbors are the boundary of standable
/// terrain and act as Dijkstra seeds for distance-from-wall.
fn wall_adjacent_cells(adj: &CsrAdjacency, idx_to_cell: &[VoxelKey]) -> Vec<u32> {
    let n = adj.n as usize;
    let mut same_z = vec![0u8; n];
    for u in 0..n {
        let iz_u = idx_to_cell[u].2;
        let lo = adj.indptr[u] as usize;
        let hi = adj.indptr[u + 1] as usize;
        for k in lo..hi {
            let v = adj.indices[k] as usize;
            if idx_to_cell[v].2 == iz_u {
                same_z[u] += 1;
            }
        }
    }
    let mut wall: Vec<u32> = (0..n as u32).filter(|&i| same_z[i as usize] < 4).collect();
    if wall.is_empty() {
        wall.push(0);
    }
    wall
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
        // Min-heap: smallest f32 pops first.
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

fn dijkstra_multi_source(adj: &CsrAdjacency, sources: &[u32]) -> Vec<f32> {
    let n = adj.n as usize;
    let mut dist = vec![f32::INFINITY; n];
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    for &s in sources {
        dist[s as usize] = 0.0;
        heap.push(Scored(0.0, s));
    }
    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist[u as usize] {
            continue;
        }
        let lo = adj.indptr[u as usize] as usize;
        let hi = adj.indptr[u as usize + 1] as usize;
        for k in lo..hi {
            let v = adj.indices[k];
            let w = adj.data[k];
            let nd = d + w;
            if nd < dist[v as usize] {
                dist[v as usize] = nd;
                heap.push(Scored(nd, v));
            }
        }
    }
    dist
}

/// Bin placed nodes by node_spacing-sized cells. For each candidate, scan the
/// 27 nearby bins for any node within Euclidean node_spacing. Replaces the
/// Python's cKDTree.
fn nms_grid(
    candidates_sorted: &[u32],
    idx_to_cell: &[VoxelKey],
    voxel_size: f32,
    node_spacing_m: f32,
) -> Vec<u32> {
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

    let mut bins: AHashMap<(i32, i32, i32), Vec<VoxelKey>> = AHashMap::new();
    let mut survivors = Vec::new();
    for &cand in candidates_sorted {
        let cell = idx_to_cell[cand as usize];
        let (bx, by, bz) = bin_of(cell);
        let mut killed = false;
        'outer: for dbx in -1..=1 {
            for dby in -1..=1 {
                for dbz in -1..=1 {
                    if let Some(nearby) = bins.get(&(bx + dbx, by + dby, bz + dbz)) {
                        for &n in nearby {
                            let dx = (cell.0 - n.0) as f64 * v;
                            let dy = (cell.1 - n.1) as f64 * v;
                            let dz = (cell.2 - n.2) as f64 * v;
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
            survivors.push(cand);
            bins.entry((bx, by, bz)).or_default().push(cell);
        }
    }
    survivors
}

/// Linear ramp from penalty=2 at wall to penalty=1 at buffer, capped at 1.
/// Per-edge multiplier averages the two endpoints' penalties.
fn wall_safe_adjacency(adj: &CsrAdjacency, dist: &[f32], buffer_m: f32) -> CsrAdjacency {
    let n = adj.n as usize;
    let penalty: Vec<f32> = (0..n)
        .map(|i| (1.0 + (buffer_m - dist[i]) / buffer_m).max(1.0))
        .collect();

    let mut data = Vec::with_capacity(adj.data.len());
    for u in 0..n {
        let lo = adj.indptr[u] as usize;
        let hi = adj.indptr[u + 1] as usize;
        let pu = penalty[u];
        for k in lo..hi {
            let v = adj.indices[k] as usize;
            let pv = penalty[v];
            data.push(adj.data[k] * (pu + pv) / 2.0);
        }
    }

    CsrAdjacency {
        indptr: adj.indptr.clone(),
        indices: adj.indices.clone(),
        data,
        n: adj.n,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VOXEL: f32 = 0.1;

    fn open_patch_5x5() -> Vec<VoxelKey> {
        let mut c = Vec::new();
        for ix in 0..5 {
            for iy in 0..5 {
                c.push((ix, iy, 0));
            }
        }
        c
    }

    fn open_patch(ix0: i32, iy0: i32, size: i32) -> Vec<VoxelKey> {
        let mut c = Vec::new();
        for dx in 0..size {
            for dy in 0..size {
                c.push((ix0 + dx, iy0 + dy, 0));
            }
        }
        c
    }

    #[test]
    fn empty_input() {
        let sg = place_nodes(&[], VOXEL, 2, 1.0, 0.3);
        assert_eq!(sg.adj.n, 0);
        assert!(sg.nodes.is_empty());
    }

    #[test]
    fn isolated_cell_places_no_node() {
        // Single cell has 0 neighbors, is wall-adjacent, dist=0, below buffer.
        let sg = place_nodes(&[(0, 0, 0)], VOXEL, 2, 1.0, 0.3);
        assert!(sg.nodes.is_empty());
    }

    #[test]
    fn open_patch_places_at_least_one_node() {
        // 10x10 at voxel=0.1 is 1m x 1m. Center is ~0.5m from any wall, well above buffer=0.3m.
        let sg = place_nodes(&open_patch(0, 0, 10), VOXEL, 2, 1.0, 0.3);
        assert!(!sg.nodes.is_empty());
        for n in &sg.nodes {
            let (ix, iy, _) = n.cell;
            assert!((0..10).contains(&ix) && (0..10).contains(&iy));
        }
    }

    #[test]
    fn nms_enforces_spacing() {
        // Two 10x10 patches separated by 1m gap; each places at least one node, no pair within 1m.
        let mut cells = open_patch(0, 0, 10);
        cells.extend(open_patch(20, 0, 10));
        let sg = place_nodes(&cells, VOXEL, 2, 1.0, 0.3);
        assert!(sg.nodes.len() >= 2);
        for i in 0..sg.nodes.len() {
            for j in (i + 1)..sg.nodes.len() {
                let a = sg.nodes[i].pos;
                let b = sg.nodes[j].pos;
                let dx = a.0 - b.0;
                let dy = a.1 - b.1;
                let dz = a.2 - b.2;
                let d_sq = dx * dx + dy * dy + dz * dz;
                assert!(d_sq > 1.0 * 1.0 - 1e-4);
            }
        }
    }

    #[test]
    fn wall_cells_scale_outbound_cost() {
        // Strip of 10 cells. End cells have 1 same-z neighbor → wall-adjacent → dist=0 → penalty=2.
        let cells: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let sg = place_nodes(&cells, VOXEL, 2, 1.0, 0.3);
        let end_idx = sg.cell_to_idx[&(0, 0, 0)] as usize;
        let lo = sg.adj.indptr[end_idx] as usize;
        let hi = sg.adj.indptr[end_idx + 1] as usize;
        assert!(hi > lo);
        // End cell penalty=2, neighbor penalty>=1, so outbound cost >= 1.5 * VOXEL.
        for k in lo..hi {
            assert!(sg.adj.data[k] >= 1.5 * VOXEL - 1e-5);
        }
    }

    #[test]
    fn dijkstra_distances_grow_from_seeds() {
        let cells = open_patch_5x5();
        let lookup = build_surface_lookup(&cells);
        let SurfaceAdjacency {
            adj, idx_to_cell, ..
        } = build_surface_adjacency(&lookup, VOXEL, 2);
        let seeds = wall_adjacent_cells(&adj, &idx_to_cell);
        let dist = dijkstra_multi_source(&adj, &seeds);

        let center = idx_to_cell.iter().position(|&c| c == (2, 2, 0)).unwrap();
        let corner = idx_to_cell.iter().position(|&c| c == (0, 0, 0)).unwrap();
        assert!(dist[center] > dist[corner]);
        assert_eq!(dist[corner], 0.0);
    }
}
