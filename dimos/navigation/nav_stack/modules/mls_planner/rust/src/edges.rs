// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node-graph edge construction.
//!
//! Run multi-source Dijkstra from every node cell to partition the surface
//! into Voronoi regions (one per node). Walk every cell-level edge that
//! straddles a region boundary; for each ordered node pair keep the cheapest
//! crossing as a node-level edge.
//!
//! Each `NodeEdge` stores only the two boundary cells and the total cost.
//! The cell-by-cell path along an edge is reconstructed lazily in plan.rs
//! by walking the saved predecessor array.

#![allow(dead_code)]

use ahash::AHashMap;

use crate::adjacency::{CsrAdjacency, SurfaceLookup};
use crate::dijkstra::{dijkstra, DijkstraResult};
use crate::nodes::{NodeData, SurfaceGraph};
use crate::voxel::VoxelKey;

pub struct NodeEdge {
    pub a: u32,
    pub b: u32,
    pub cost: f32,
    /// Cell on a's side of the cheapest Voronoi boundary crossing.
    pub boundary_u: u32,
    /// Cell on b's side.
    pub boundary_v: u32,
}

pub struct PlannerGraph {
    pub adj: CsrAdjacency,
    pub idx_to_cell: Vec<VoxelKey>,
    pub cell_to_idx: AHashMap<VoxelKey, u32>,
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
    pub node_edges: Vec<NodeEdge>,
    /// node_id -> indices into `node_edges`. Adjacency for the node-level
    /// shortest-path query in Stage D.
    pub node_adj: Vec<Vec<u32>>,
    /// Per-cell predecessor along the multi-source Dijkstra from node seeds.
    /// -1 marks source cells and unreachable cells. Used by plan.rs to walk
    /// cell paths for the edges on a chosen route.
    pub cell_predecessors: Vec<i32>,
}

pub fn add_node_edges(sg: SurfaceGraph) -> PlannerGraph {
    let SurfaceGraph {
        adj,
        idx_to_cell,
        cell_to_idx,
        surface_lookup,
        nodes,
    } = sg;

    if nodes.is_empty() {
        let n = adj.n as usize;
        return PlannerGraph {
            adj,
            idx_to_cell,
            cell_to_idx,
            surface_lookup,
            nodes,
            node_edges: Vec::new(),
            node_adj: Vec::new(),
            cell_predecessors: vec![-1; n],
        };
    }

    let source_indices: Vec<u32> = nodes.iter().map(|n| cell_to_idx[&n.cell]).collect();
    let DijkstraResult { dist, pred, source } = dijkstra(&adj, &source_indices);
    let node_edges = best_boundary_edges(&adj, &dist, &source);

    let mut node_adj: Vec<Vec<u32>> = vec![Vec::new(); nodes.len()];
    for (edge_idx, edge) in node_edges.iter().enumerate() {
        node_adj[edge.a as usize].push(edge_idx as u32);
        node_adj[edge.b as usize].push(edge_idx as u32);
    }

    PlannerGraph {
        adj,
        idx_to_cell,
        cell_to_idx,
        surface_lookup,
        nodes,
        node_edges,
        node_adj,
        cell_predecessors: pred,
    }
}

fn best_boundary_edges(adj: &CsrAdjacency, dist: &[f32], source: &[i32]) -> Vec<NodeEdge> {
    let mut best: AHashMap<(u32, u32), NodeEdge> = AHashMap::new();

    for u in 0..adj.n as usize {
        let sa = source[u];
        if sa < 0 {
            continue;
        }
        let lo = adj.indptr[u] as usize;
        let hi = adj.indptr[u + 1] as usize;
        for k in lo..hi {
            let v = adj.indices[k] as usize;
            let sb = source[v];
            if sb < 0 || sa == sb {
                continue;
            }
            let edge_w = adj.data[k];
            let cost = dist[u] + edge_w + dist[v];

            let (key_a, key_b, bu, bv) = if (sa as u32) < (sb as u32) {
                (sa as u32, sb as u32, u as u32, v as u32)
            } else {
                (sb as u32, sa as u32, v as u32, u as u32)
            };

            let entry = best.entry((key_a, key_b)).or_insert(NodeEdge {
                a: key_a,
                b: key_b,
                cost: f32::INFINITY,
                boundary_u: 0,
                boundary_v: 0,
            });
            if cost < entry.cost {
                entry.cost = cost;
                entry.boundary_u = bu;
                entry.boundary_v = bv;
            }
        }
    }

    let mut out: Vec<NodeEdge> = best.into_values().collect();
    out.sort_by_key(|e| (e.a, e.b));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_adjacency, build_surface_lookup, SurfaceAdjacency};
    use crate::voxel::surface_point_xyz;

    const VOXEL: f32 = 0.1;

    /// Build a SurfaceGraph from a list of surface cells and a list of node
    /// cells (which must be a subset of the surface cells). Bypasses
    /// place_nodes so the test author controls which cells become nodes.
    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> SurfaceGraph {
        let surface_lookup = build_surface_lookup(surface_cells);
        let SurfaceAdjacency {
            adj,
            idx_to_cell,
            cell_to_idx,
        } = build_surface_adjacency(&surface_lookup, VOXEL, 2);
        let nodes: Vec<NodeData> = node_cells
            .iter()
            .map(|&c| NodeData {
                cell: c,
                pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
            })
            .collect();
        SurfaceGraph {
            adj,
            idx_to_cell,
            cell_to_idx,
            surface_lookup,
            nodes,
        }
    }

    /// 20-cell strip along x at iz=0.
    fn strip_cells() -> Vec<VoxelKey> {
        (0..20).map(|x| (x, 0, 0)).collect()
    }

    #[test]
    fn no_nodes_yields_no_edges() {
        let sg = graph_with_nodes(&strip_cells(), &[]);
        let pg = add_node_edges(sg);
        assert!(pg.node_edges.is_empty());
        assert!(pg.node_adj.is_empty());
    }

    #[test]
    fn single_node_has_no_edges() {
        let sg = graph_with_nodes(&strip_cells(), &[(10, 0, 0)]);
        let pg = add_node_edges(sg);
        assert!(pg.node_edges.is_empty());
        assert_eq!(pg.node_adj.len(), 1);
        assert!(pg.node_adj[0].is_empty());
    }

    #[test]
    fn two_nodes_on_strip_have_one_edge() {
        let sg = graph_with_nodes(&strip_cells(), &[(3, 0, 0), (15, 0, 0)]);
        let pg = add_node_edges(sg);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];
        assert_eq!((e.a, e.b), (0, 1));
        assert_eq!(pg.node_adj[0], vec![0]);
        assert_eq!(pg.node_adj[1], vec![0]);
    }

    #[test]
    fn three_nodes_in_line_form_a_chain() {
        // Nodes at 3, 10, 17 in a strip 0..20. Voronoi boundaries are
        // around 6-7 and 13-14, so we get edges (0,1) and (1,2) but no (0,2).
        let sg = graph_with_nodes(&strip_cells(), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let pg = add_node_edges(sg);
        let pairs: Vec<(u32, u32)> = pg.node_edges.iter().map(|e| (e.a, e.b)).collect();
        assert_eq!(pairs, vec![(0, 1), (1, 2)]);
    }

    #[test]
    fn disconnected_components_have_no_edge() {
        // Two strips with a gap, one node in each.
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let sg = graph_with_nodes(&cells, &[(2, 0, 0), (12, 0, 0)]);
        let pg = add_node_edges(sg);
        assert!(pg.node_edges.is_empty());
    }

    #[test]
    fn predecessor_walk_recovers_cell_path() {
        // Two nodes at strip ends. Walk preds from each boundary cell back to
        // its owning node cell and verify the chain reaches the node.
        let sg = graph_with_nodes(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let pg = add_node_edges(sg);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];

        let cell_a = pg.cell_to_idx[&pg.nodes[0].cell];
        let cell_b = pg.cell_to_idx[&pg.nodes[1].cell];

        let mut cur = e.boundary_u as i32;
        let mut hops = 0;
        while pg.cell_predecessors[cur as usize] >= 0 {
            cur = pg.cell_predecessors[cur as usize];
            hops += 1;
            assert!(hops < 1000, "predecessor walk did not terminate");
        }
        assert_eq!(cur as u32, cell_a, "u-side preds must reach node a");

        let mut cur = e.boundary_v as i32;
        while pg.cell_predecessors[cur as usize] >= 0 {
            cur = pg.cell_predecessors[cur as usize];
        }
        assert_eq!(cur as u32, cell_b, "v-side preds must reach node b");
    }
}
