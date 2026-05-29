// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Per-column surface lookup and 4-connected adjacency over surface cells.
//!
//! Wall-safe cost smoothing in nodes.rs should make paths equivalent.

#![allow(dead_code)] // consumed incrementally by later stage modules

use ahash::AHashMap;

use crate::voxel::VoxelKey;

pub type SurfaceLookup = AHashMap<(i32, i32), Vec<i32>>;

const NEIGHBORS_4: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

pub struct CsrAdjacency {
    pub indptr: Vec<u32>,
    pub indices: Vec<u32>,
    pub data: Vec<f32>,
    pub n: u32,
}

pub struct SurfaceAdjacency {
    pub adj: CsrAdjacency,
    pub idx_to_cell: Vec<VoxelKey>,
    pub cell_to_idx: AHashMap<VoxelKey, u32>,
}

/// Group cells by XY column with sorted unique iz per column.
pub fn build_surface_lookup(cells: &[VoxelKey]) -> SurfaceLookup {
    let mut lookup: SurfaceLookup = AHashMap::new();
    for &(ix, iy, iz) in cells {
        lookup.entry((ix, iy)).or_default().push(iz);
    }
    for zs in lookup.values_mut() {
        zs.sort_unstable();
        zs.dedup();
    }
    lookup
}

/// 4-connected XY adjacency with per-step dz cap. Cell ordering is
/// lex-sorted by column then iz so the output is deterministic across runs.
pub fn build_surface_adjacency(
    surface_lookup: &SurfaceLookup,
    voxel_size: f32,
    step_threshold_cells: i32,
) -> SurfaceAdjacency {
    let mut columns: Vec<(i32, i32)> = surface_lookup.keys().copied().collect();
    columns.sort_unstable();

    let mut idx_to_cell: Vec<VoxelKey> = Vec::new();
    for &(ix, iy) in &columns {
        for &iz in &surface_lookup[&(ix, iy)] {
            idx_to_cell.push((ix, iy, iz));
        }
    }
    let n = idx_to_cell.len();
    let cell_to_idx: AHashMap<VoxelKey, u32> = idx_to_cell
        .iter()
        .enumerate()
        .map(|(i, &c)| (c, i as u32))
        .collect();

    let mut indptr: Vec<u32> = Vec::with_capacity(n + 1);
    indptr.push(0);
    let mut indices: Vec<u32> = Vec::new();
    let mut data: Vec<f32> = Vec::new();

    for &(ix, iy, iz) in &idx_to_cell {
        for (dx, dy) in NEIGHBORS_4 {
            let Some(zs) = surface_lookup.get(&(ix + dx, iy + dy)) else {
                continue;
            };
            for &nz in zs {
                let dz = nz - iz;
                if dz.abs() > step_threshold_cells {
                    continue;
                }
                let dst = cell_to_idx[&(ix + dx, iy + dy, nz)];
                let cost = ((dx * dx + dy * dy + dz * dz) as f32).sqrt() * voxel_size;
                indices.push(dst);
                data.push(cost);
            }
        }
        indptr.push(indices.len() as u32);
    }

    SurfaceAdjacency {
        adj: CsrAdjacency {
            indptr,
            indices,
            data,
            n: n as u32,
        },
        idx_to_cell,
        cell_to_idx,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VOXEL: f32 = 0.1;

    fn approx_eq(a: f32, b: f32) {
        let eps = 1e-5;
        assert!((a - b).abs() < eps, "{a} != {b} (eps {eps})");
    }

    fn edges(sa: &SurfaceAdjacency) -> Vec<(VoxelKey, VoxelKey, f32)> {
        let mut out = Vec::new();
        for src in 0..sa.adj.n as usize {
            let lo = sa.adj.indptr[src] as usize;
            let hi = sa.adj.indptr[src + 1] as usize;
            for k in lo..hi {
                let dst = sa.adj.indices[k] as usize;
                out.push((sa.idx_to_cell[src], sa.idx_to_cell[dst], sa.adj.data[k]));
            }
        }
        out
    }

    #[test]
    fn empty_input_yields_empty_adjacency() {
        let lookup = build_surface_lookup(&[]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(sa.adj.n, 0);
        assert_eq!(sa.adj.indptr, vec![0]);
        assert!(sa.adj.indices.is_empty());
        assert!(sa.adj.data.is_empty());
        assert!(sa.idx_to_cell.is_empty());
    }

    #[test]
    fn single_cell_has_no_edges() {
        let lookup = build_surface_lookup(&[(0, 0, 0)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(sa.adj.n, 1);
        assert_eq!(sa.adj.indptr, vec![0, 0]);
        assert!(sa.adj.indices.is_empty());
    }

    #[test]
    fn same_z_neighbors_are_bidirectional() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 0)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(sa.adj.indices.len(), 2);
        for e in edges(&sa) {
            approx_eq(e.2, VOXEL);
        }
    }

    #[test]
    fn diagonal_not_connected_under_4_connectivity() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 1, 0)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert!(
            sa.adj.indices.is_empty(),
            "diagonal must not connect under 4-connectivity"
        );
    }

    #[test]
    fn step_threshold_blocks_large_dz() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 5)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert!(
            sa.adj.indices.is_empty(),
            "dz=5 must not connect when step_threshold=2"
        );
    }

    #[test]
    fn step_within_threshold_uses_3d_distance() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 1)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(sa.adj.indices.len(), 2);
        let expected = (2.0_f32).sqrt() * VOXEL;
        for e in edges(&sa) {
            approx_eq(e.2, expected);
        }
    }

    #[test]
    fn same_column_cells_are_not_self_connected() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (0, 0, 5)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 10);
        assert!(sa.adj.indices.is_empty());
    }

    #[test]
    fn plus_pattern_center_has_four_neighbors() {
        let cells = vec![(0, 0, 0), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)];
        let lookup = build_surface_lookup(&cells);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        let center_idx = sa.cell_to_idx[&(0, 0, 0)] as usize;
        let lo = sa.adj.indptr[center_idx] as usize;
        let hi = sa.adj.indptr[center_idx + 1] as usize;
        assert_eq!(hi - lo, 4);
    }

    #[test]
    fn deduplicates_repeated_cells() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (0, 0, 0), (1, 0, 0)]);
        let sa = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(sa.adj.n, 2);
    }
}
