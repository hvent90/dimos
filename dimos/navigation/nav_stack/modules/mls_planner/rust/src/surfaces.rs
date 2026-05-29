// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Surface extraction: voxelize a cloud, mark cells with robot-height clearance
//! above as standable, then morphologically close per-z-level holes without
//! letting closing bridge across walls.

#![allow(dead_code)] // consumed incrementally

use ahash::{AHashMap, AHashSet};
use image::{GrayImage, Luma};
use imageproc::distance_transform::Norm;
use imageproc::morphology::{dilate, erode};

use crate::voxel::VoxelKey;

const ON: Luma<u8> = Luma([255]);
const OFF: Luma<u8> = Luma([0]);

#[inline]
fn voxelize(p: (f32, f32, f32), voxel_size: f32) -> VoxelKey {
    let inv = 1.0 / voxel_size;
    (
        (p.0 * inv).floor() as i32,
        (p.1 * inv).floor() as i32,
        (p.2 * inv).floor() as i32,
    )
}

/// A cell (ix, iy, iz) is standable iff its column has no obstacle
/// within the height of the robot.
fn is_standable(
    ix: i32,
    iy: i32,
    iz: i32,
    obstacles: &AHashSet<VoxelKey>,
    height_cells: i32,
) -> bool {
    for dz in 1..=height_cells {
        if obstacles.contains(&(ix, iy, iz + dz)) {
            return false;
        }
    }
    true
}

/// Extract standable cells from a point cloud, then close small holes.
///
/// Returns cell indices.
pub fn extract_surfaces(
    points: &[(f32, f32, f32)],
    voxel_size: f32,
    height_cells: i32,
    dilation_passes: u32,
    erosion_passes: u32,
) -> Vec<VoxelKey> {
    if points.is_empty() {
        return Vec::new();
    }

    let obstacles: AHashSet<VoxelKey> = points.iter().map(|&p| voxelize(p, voxel_size)).collect();

    let mut by_col: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
    for &(ix, iy, iz) in &obstacles {
        by_col.entry((ix, iy)).or_default().push(iz);
    }

    let mut standable: Vec<VoxelKey> = Vec::new();
    for ((ix, iy), zs) in by_col.iter_mut() {
        zs.sort_unstable();
        for w in zs.windows(2) {
            if w[1] - w[0] > height_cells {
                standable.push((*ix, *iy, w[0]));
            }
        }
        if let Some(&last_iz) = zs.last() {
            standable.push((*ix, *iy, last_iz));
        }
    }

    close_surface_holes(
        standable,
        &obstacles,
        dilation_passes,
        erosion_passes,
        height_cells,
    )
}

fn close_surface_holes(
    standable: Vec<VoxelKey>,
    obstacles: &AHashSet<VoxelKey>,
    dilation_passes: u32,
    erosion_passes: u32,
    height_cells: i32,
) -> Vec<VoxelKey> {
    if standable.is_empty() || (dilation_passes == 0 && erosion_passes == 0) {
        return standable;
    }

    let mut by_z: AHashMap<i32, Vec<(i32, i32)>> = AHashMap::new();
    for &(ix, iy, iz) in &standable {
        by_z.entry(iz).or_default().push((ix, iy));
    }

    let mut out = Vec::new();
    for (iz, xys) in by_z {
        out.extend(close_at_z(
            &xys,
            iz,
            obstacles,
            dilation_passes,
            erosion_passes,
            height_cells,
        ));
    }
    out
}

fn close_at_z(
    xys: &[(i32, i32)],
    iz: i32,
    obstacles: &AHashSet<VoxelKey>,
    dilation_passes: u32,
    erosion_passes: u32,
    height_cells: i32,
) -> Vec<VoxelKey> {
    let pad = dilation_passes as i32;
    let mut min_x = i32::MAX;
    let mut max_x = i32::MIN;
    let mut min_y = i32::MAX;
    let mut max_y = i32::MIN;
    for &(ix, iy) in xys {
        min_x = min_x.min(ix);
        max_x = max_x.max(ix);
        min_y = min_y.min(iy);
        max_y = max_y.max(iy);
    }

    let w = (max_x - min_x + 1 + 2 * pad) as u32;
    let h = (max_y - min_y + 1 + 2 * pad) as u32;
    let x0 = min_x - pad;
    let y0 = min_y - pad;

    let mut img = GrayImage::from_pixel(w, h, OFF);
    for &(ix, iy) in xys {
        img.put_pixel((ix - x0) as u32, (iy - y0) as u32, ON);
    }

    // L1 norm for 4 neighbor connectivity
    if dilation_passes > 0 {
        img = dilate(&img, Norm::L1, dilation_passes as u8);
    }
    if erosion_passes > 0 {
        img = erode(&img, Norm::L1, erosion_passes as u8);
    }

    let mut out = Vec::new();
    for py in 0..h {
        for px in 0..w {
            if img.get_pixel(px, py).0[0] == 0 {
                continue;
            }
            let ix = x0 + px as i32;
            let iy = y0 + py as i32;
            if !is_standable(ix, iy, iz, obstacles, height_cells) {
                continue;
            }
            out.push((ix, iy, iz));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input() {
        assert!(extract_surfaces(&[], 1.0, 5, 0, 0).is_empty());
    }

    #[test]
    fn single_point_is_topmost_surface() {
        let s = extract_surfaces(&[(0.5, 0.5, 0.5)], 1.0, 5, 0, 0);
        assert_eq!(s, vec![(0, 0, 0)]);
    }

    #[test]
    fn stacked_points_within_headroom_only_topmost_is_surface() {
        let pts: Vec<(f32, f32, f32)> = (0..5).map(|z| (0.5, 0.5, z as f32 + 0.5)).collect();
        let s = extract_surfaces(&pts, 1.0, 5, 0, 0);
        assert_eq!(s, vec![(0, 0, 4)]);
    }

    #[test]
    fn gap_larger_than_headroom_makes_lower_cell_standable() {
        // Obstacles at iz=0 and iz=10 with height_cells=5. Lower cell has gap=10 > 5.
        let pts: Vec<(f32, f32, f32)> = vec![(0.5, 0.5, 0.5), (0.5, 0.5, 10.5)];
        let mut s = extract_surfaces(&pts, 1.0, 5, 0, 0);
        s.sort();
        assert_eq!(s, vec![(0, 0, 0), (0, 0, 10)]);
    }

    #[test]
    fn morphological_closing_fills_center_hole() {
        // Ring of 8 cells around (0,0) at iz=0, no obstacles above.
        let mut pts: Vec<(f32, f32, f32)> = Vec::new();
        for (dx, dy) in [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ] {
            pts.push((dx as f32 + 0.5, dy as f32 + 0.5, 0.5));
        }
        let s = extract_surfaces(&pts, 1.0, 5, 3, 3);
        assert!(
            s.contains(&(0, 0, 0)),
            "closing should fill the center hole"
        );
    }

    #[test]
    fn closing_does_not_bridge_obstacle_in_headroom() {
        // Ring of 8 cells at iz=0 + an obstacle directly above the hole at (0,0,1).
        // The hole at (0,0,0) is vetoed because its headroom column has an obstacle.
        let mut pts: Vec<(f32, f32, f32)> = Vec::new();
        for (dx, dy) in [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ] {
            pts.push((dx as f32 + 0.5, dy as f32 + 0.5, 0.5));
        }
        pts.push((0.5, 0.5, 1.5));
        let s = extract_surfaces(&pts, 1.0, 5, 3, 3);
        assert!(!s.contains(&(0, 0, 0)));
    }
}
