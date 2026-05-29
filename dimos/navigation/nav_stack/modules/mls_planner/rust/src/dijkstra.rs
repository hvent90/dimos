// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Multi-source Dijkstra over the cell-keyed surface adjacency.
//!
//! Tracks distance, predecessor, and nearest-source label for each reached
//! cell. Cells absent from the result are unreachable from every source.

#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::AHashMap;

use crate::adjacency::SurfaceAdjacency;
use crate::voxel::VoxelKey;

pub struct DijkstraResult {
    pub dist: AHashMap<VoxelKey, f32>,
    /// Predecessor cell along the shortest path back to a source.
    pub pred: AHashMap<VoxelKey, VoxelKey>,
    /// Id of the cheapest source to return to
    pub source: AHashMap<VoxelKey, u32>,
}

pub fn dijkstra(adj: &SurfaceAdjacency, sources: &[VoxelKey]) -> DijkstraResult {
    let mut dist: AHashMap<VoxelKey, f32> = AHashMap::new();
    let mut pred: AHashMap<VoxelKey, VoxelKey> = AHashMap::new();
    let mut source: AHashMap<VoxelKey, u32> = AHashMap::new();
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();

    for (label, &s) in sources.iter().enumerate() {
        if !adj.contains(s) {
            continue;
        }
        dist.insert(s, 0.0);
        source.insert(s, label as u32);
        heap.push(Scored(0.0, s));
    }

    while let Some(Scored(d, u)) = heap.pop() {
        if let Some(&cur) = dist.get(&u) {
            if d > cur {
                continue;
            }
        }
        let su = source[&u];
        for edge in adj.neighbors(u) {
            let nd = d + edge.cost;
            let should_update = match dist.get(&edge.dst) {
                None => true,
                Some(&cur) => nd < cur,
            };
            if should_update {
                dist.insert(edge.dst, nd);
                pred.insert(edge.dst, u);
                source.insert(edge.dst, su);
                heap.push(Scored(nd, edge.dst));
            }
        }
    }

    DijkstraResult { dist, pred, source }
}

struct Scored(f32, VoxelKey);

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

    fn chain(n: i32) -> SurfaceAdjacency {
        let mut adj = SurfaceAdjacency::new();
        for i in 0..n {
            adj.add_cell((i, 0, 0));
        }
        for i in 0..n - 1 {
            adj.add_edge((i, 0, 0), (i + 1, 0, 0), 1.0);
            adj.add_edge((i + 1, 0, 0), (i, 0, 0), 1.0);
        }
        adj
    }

    #[test]
    fn empty_sources_leaves_everything_unreachable() {
        let adj = chain(4);
        let r = dijkstra(&adj, &[]);
        assert!(r.dist.is_empty());
        assert!(r.pred.is_empty());
        assert!(r.source.is_empty());
    }

    #[test]
    fn single_source_dist_and_pred() {
        let adj = chain(5);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        for i in 0..5 {
            assert_eq!(r.dist[&(i, 0, 0)], i as f32);
            assert_eq!(r.source[&(i, 0, 0)], 0);
        }
        assert!(!r.pred.contains_key(&(0, 0, 0)));
        let mut cur = (4, 0, 0);
        let mut hops = 0;
        while let Some(&p) = r.pred.get(&cur) {
            cur = p;
            hops += 1;
        }
        assert_eq!(cur, (0, 0, 0));
        assert_eq!(hops, 4);
    }

    #[test]
    fn multi_source_labels_by_nearest() {
        // Sources at 0 and 4 on a 5-cell chain. Cells 0-1 closer to source 0,
        // cells 3-4 closer to source 1. Cell 2 is equidistant.
        let adj = chain(5);
        let r = dijkstra(&adj, &[(0, 0, 0), (4, 0, 0)]);
        assert_eq!(r.source[&(0, 0, 0)], 0);
        assert_eq!(r.source[&(1, 0, 0)], 0);
        assert_eq!(r.source[&(3, 0, 0)], 1);
        assert_eq!(r.source[&(4, 0, 0)], 1);
        // Tie at cell 2 must resolve to one of the two sources.
        let s2 = r.source[&(2, 0, 0)];
        assert!(s2 == 0 || s2 == 1);
        assert_eq!(r.dist[&(0, 0, 0)], 0.0);
        assert_eq!(r.dist[&(1, 0, 0)], 1.0);
        assert_eq!(r.dist[&(2, 0, 0)], 2.0);
        assert_eq!(r.dist[&(3, 0, 0)], 1.0);
        assert_eq!(r.dist[&(4, 0, 0)], 0.0);
    }

    #[test]
    fn disconnected_cells_stay_unreachable() {
        // Two separate chains 0-1 and 2-3, source at 0.
        let mut adj = SurfaceAdjacency::new();
        for &c in &[(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)] {
            adj.add_cell(c);
        }
        adj.add_edge((0, 0, 0), (1, 0, 0), 1.0);
        adj.add_edge((1, 0, 0), (0, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (3, 0, 0), 1.0);
        adj.add_edge((3, 0, 0), (2, 0, 0), 1.0);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        assert_eq!(r.dist[&(0, 0, 0)], 0.0);
        assert_eq!(r.dist[&(1, 0, 0)], 1.0);
        assert!(!r.dist.contains_key(&(2, 0, 0)));
        assert!(!r.dist.contains_key(&(3, 0, 0)));
        assert!(!r.source.contains_key(&(2, 0, 0)));
        assert!(!r.pred.contains_key(&(2, 0, 0)));
    }

    #[test]
    fn shorter_path_overrides_longer() {
        // 0 - 1 with cost 10, 0 - 2 - 1 with cost 1+1=2.
        let mut adj = SurfaceAdjacency::new();
        for &c in &[(0, 0, 0), (1, 0, 0), (2, 0, 0)] {
            adj.add_cell(c);
        }
        adj.add_edge((0, 0, 0), (1, 0, 0), 10.0);
        adj.add_edge((1, 0, 0), (0, 0, 0), 10.0);
        adj.add_edge((0, 0, 0), (2, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (0, 0, 0), 1.0);
        adj.add_edge((2, 0, 0), (1, 0, 0), 1.0);
        adj.add_edge((1, 0, 0), (2, 0, 0), 1.0);
        let r = dijkstra(&adj, &[(0, 0, 0)]);
        assert_eq!(r.dist[&(1, 0, 0)], 2.0);
        assert_eq!(r.pred[&(1, 0, 0)], (2, 0, 0));
    }
}
