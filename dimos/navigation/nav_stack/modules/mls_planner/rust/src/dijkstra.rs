// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Multi-source Dijkstra over the CSR adjacency.
//!
//! Always tracks distance, predecessors, and per-cell source labels. Some
//! callers only need `dist` (e.g. wall-distance in nodes.rs) and ignore the
//! rest. The extra arrays cost two Vec<i32> allocations and two writes per
//! heap pop, which is negligible against the algorithmic work.

#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::adjacency::CsrAdjacency;

pub struct DijkstraResult {
    pub dist: Vec<f32>,
    /// Predecessor cell along the shortest path back to a source. -1 marks
    /// source cells and unreachable cells. Used downstream to reconstruct
    /// cell-by-cell paths lazily.
    pub pred: Vec<i32>,
    /// Index into the caller's `sources` slice. When the caller passes node
    /// cells in node-id order this doubles as the nearest-node id, which is
    /// the Voronoi partition. -1 for unreachable cells.
    pub source: Vec<i32>,
}

pub fn dijkstra(adj: &CsrAdjacency, sources: &[u32]) -> DijkstraResult {
    let n = adj.n as usize;
    let mut dist = vec![f32::INFINITY; n];
    let mut pred = vec![-1i32; n];
    let mut source = vec![-1i32; n];
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();

    for (label, &s) in sources.iter().enumerate() {
        let su = s as usize;
        dist[su] = 0.0;
        source[su] = label as i32;
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
            let nd = d + adj.data[k];
            if nd < dist[v as usize] {
                dist[v as usize] = nd;
                pred[v as usize] = u as i32;
                source[v as usize] = source[u as usize];
                heap.push(Scored(nd, v));
            }
        }
    }

    DijkstraResult { dist, pred, source }
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
        // Min-heap on f32 score.
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a CSR from a Vec of (src, dst, cost) edges over n cells.
    /// Edges are kept directed; caller emits both directions for an undirected graph.
    fn csr(n: u32, mut edges: Vec<(u32, u32, f32)>) -> CsrAdjacency {
        edges.sort_by_key(|&(s, _, _)| s);
        let mut indptr = vec![0u32; (n + 1) as usize];
        for &(s, _, _) in &edges {
            indptr[s as usize + 1] += 1;
        }
        for i in 1..indptr.len() {
            indptr[i] += indptr[i - 1];
        }
        let indices: Vec<u32> = edges.iter().map(|&(_, d, _)| d).collect();
        let data: Vec<f32> = edges.iter().map(|&(_, _, w)| w).collect();
        CsrAdjacency {
            indptr,
            indices,
            data,
            n,
        }
    }

    /// Bidirectional chain 0 - 1 - 2 - 3 - 4 with unit edge cost.
    fn chain(n: u32) -> CsrAdjacency {
        let mut edges = Vec::new();
        for i in 0..n - 1 {
            edges.push((i, i + 1, 1.0));
            edges.push((i + 1, i, 1.0));
        }
        csr(n, edges)
    }

    #[test]
    fn empty_sources_leaves_everything_unreachable() {
        let adj = chain(4);
        let r = dijkstra(&adj, &[]);
        for i in 0..4 {
            assert!(r.dist[i].is_infinite());
            assert_eq!(r.pred[i], -1);
            assert_eq!(r.source[i], -1);
        }
    }

    #[test]
    fn single_source_dist_and_pred() {
        let adj = chain(5);
        let r = dijkstra(&adj, &[0]);
        assert_eq!(r.dist, vec![0.0, 1.0, 2.0, 3.0, 4.0]);
        assert_eq!(r.source, vec![0, 0, 0, 0, 0]);
        // Predecessor chain walks back to the source.
        assert_eq!(r.pred[0], -1);
        let mut cur: i32 = 4;
        let mut hops = 0;
        while r.pred[cur as usize] >= 0 {
            cur = r.pred[cur as usize];
            hops += 1;
        }
        assert_eq!(cur, 0);
        assert_eq!(hops, 4);
    }

    #[test]
    fn multi_source_labels_by_nearest() {
        // Sources at 0 and 4 on a 5-cell chain. Cells 0-1 closer to source 0,
        // cells 3-4 closer to source 1. Cell 2 is equidistant.
        let adj = chain(5);
        let r = dijkstra(&adj, &[0, 4]);
        assert_eq!(r.source[0], 0);
        assert_eq!(r.source[1], 0);
        assert_eq!(r.source[3], 1);
        assert_eq!(r.source[4], 1);
        // Cell 2 labeling is implementation-defined for ties but must be one of the two.
        assert!(r.source[2] == 0 || r.source[2] == 1);
        assert_eq!(r.dist, vec![0.0, 1.0, 2.0, 1.0, 0.0]);
    }

    #[test]
    fn disconnected_cells_stay_unreachable() {
        // Two separate chains 0-1 and 2-3, source at 0.
        let edges = vec![(0, 1, 1.0), (1, 0, 1.0), (2, 3, 1.0), (3, 2, 1.0)];
        let adj = csr(4, edges);
        let r = dijkstra(&adj, &[0]);
        assert_eq!(r.dist[0], 0.0);
        assert_eq!(r.dist[1], 1.0);
        assert!(r.dist[2].is_infinite());
        assert!(r.dist[3].is_infinite());
        assert_eq!(r.source[2], -1);
        assert_eq!(r.source[3], -1);
        assert_eq!(r.pred[2], -1);
        assert_eq!(r.pred[3], -1);
    }

    #[test]
    fn shorter_path_overrides_longer() {
        // 0 - 1 with cost 10, 0 - 2 - 1 with cost 1+1=2.
        let edges = vec![
            (0, 1, 10.0),
            (1, 0, 10.0),
            (0, 2, 1.0),
            (2, 0, 1.0),
            (2, 1, 1.0),
            (1, 2, 1.0),
        ];
        let adj = csr(3, edges);
        let r = dijkstra(&adj, &[0]);
        assert_eq!(r.dist[1], 2.0);
        assert_eq!(r.pred[1], 2); // came via 2, not directly
    }
}
