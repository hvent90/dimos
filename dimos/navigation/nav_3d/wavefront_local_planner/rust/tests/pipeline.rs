// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! End-to-end pipeline tests on synthetic worlds — the guards that would have
//! caught hl58 (voxel-spaced input under-sampling the grid -> lethal blanket
//! -> robot boxed in -> empty plans at 60 Hz for a whole course).

use dimos_wavefront::costmap::{self, CostmapConfig};
use dimos_wavefront::solver::{self, SolverConfig};

/// Ground plane sampled at VOXEL spacing (0.1 m) like the terrain mapper's
/// output — twice as coarse as nothing, exactly as coarse as reality.
fn voxel_ground(half: f32, z: f32) -> Vec<[f32; 3]> {
    let mut pts = Vec::new();
    let mut x = -half;
    while x <= half {
        let mut y = -half;
        while y <= half {
            pts.push([x, y, z]);
            y += 0.1;
        }
        x += 0.1;
    }
    pts
}

fn arc(poses: &[(f32, f32, f32)]) -> f32 {
    poses
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .sum()
}

#[test]
fn flat_ground_plans_toward_the_goal() {
    let cfg = CostmapConfig::default();
    let scfg = SolverConfig::default();
    let pts = voxel_ground(8.0, 0.0);
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    let route = vec![(0.0, 0.0), (6.0, 0.0)];
    let plan = solver::plan(&map, &route, (0.0, 0.0, 0.0), 1.0, None, &scfg);
    assert!(
        plan.poses.len() >= 2,
        "flat ground must produce a plan (got {} poses — the hl58 boxed-in failure)",
        plan.poses.len()
    );
    assert!(arc(&plan.poses) > 2.0, "plan should extend toward the goal");
    let last = plan.poses.last().unwrap();
    assert!(
        last.0 > 2.0,
        "plan should head +x toward the goal, ended at {:?}",
        last
    );
}

#[test]
fn wall_forces_a_detour_not_a_beeline() {
    let cfg = CostmapConfig::default();
    let scfg = SolverConfig::default();
    let mut pts = voxel_ground(8.0, 0.0);
    // A wall at x=2, y in [-2, 2], 2 m tall (in-band vertical returns).
    let mut y = -2.0;
    while y <= 2.0 {
        let mut z = 0.0;
        while z <= 1.4 {
            pts.push([2.0, y, z]);
            z += 0.1;
        }
        y += 0.05;
    }
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    let route = vec![(0.0, 0.0), (5.0, 0.0)];
    let plan = solver::plan(&map, &route, (0.0, 0.0, 0.0), 1.0, None, &scfg);
    assert!(
        plan.poses.len() >= 2,
        "wall world must still produce a plan"
    );
    // No pose may sit inside the wall's inflated zone at the centreline.
    for p in &plan.poses {
        let near_wall = (p.0 - 2.0).abs() < 0.25 && p.1.abs() < 2.0;
        assert!(!near_wall, "plan crosses the wall at {:?}", p);
    }
}

#[test]
fn solve_is_fast_enough_for_60hz() {
    let cfg = CostmapConfig::default();
    let scfg = SolverConfig::default();
    let pts = voxel_ground(8.0, 0.0);
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    let route = vec![(0.0, 0.0), (6.0, 0.0)];
    let start = std::time::Instant::now();
    let mut prev: Option<Vec<(f32, f32)>> = None;
    const N: u32 = 60;
    for _ in 0..N {
        let plan = solver::plan(&map, &route, (0.0, 0.0, 0.0), 1.0, prev.as_deref(), &scfg);
        prev = Some(plan.poses.iter().map(|p| (p.0, p.1)).collect());
    }
    let per_solve = start.elapsed().as_secs_f64() / N as f64;
    assert!(
        per_solve < 1.0 / 60.0,
        "solve too slow for 60 Hz: {:.2} ms",
        per_solve * 1e3
    );
    // Costmap build budget: it only runs at the terrain rate (~2 Hz) but must
    // never eat a whole tick era.
    let start = std::time::Instant::now();
    let _ = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    assert!(
        start.elapsed().as_secs_f64() < 0.05,
        "costmap build too slow"
    );
}

#[test]
fn debug_flat_ground_costmap_stats() {
    let cfg = CostmapConfig::default();
    let pts = voxel_ground(8.0, 0.0);
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    let n = map.cost.len();
    let unknown = map.cost.iter().filter(|&&c| c < 0).count();
    let lethal = map.cost.iter().filter(|&&c| c >= 50).count();
    let free = n - unknown - lethal;
    println!(
        "n={} free={} lethal={} unknown={}",
        n, free, lethal, unknown
    );
    let (r, c) = map.cell(0.0, 0.0).unwrap();
    let i = r * map.width + c;
    println!(
        "robot cell cost={} dist={:.2}",
        map.cost[i], map.distance[i]
    );
    for dr in -3..=3i32 {
        let mut line = String::new();
        for dc in -3..=3i32 {
            let j = ((r as i32 + dr) as usize) * map.width + (c as i32 + dc) as usize;
            line.push_str(&format!("{:4}/{:4.2} ", map.cost[j], map.distance[j]));
        }
        println!("{}", line);
    }
}

#[test]
fn resample_survives_float_edge() {
    // Regression: resample walked past the last segment when the final target
    // exceeded the stored total by float noise (hl59 main-task panic).
    // Exercised through the public API: a route change through the debounce
    // path calls route_deviation -> resample. Reconstructed here directly on
    // the solver's densify path with awkward segment lengths.
    let cfg = SolverConfig::default();
    let map = costmap::build(&voxel_ground(8.0, 0.0), (0.0, 0.0, 0.4), 0.4, &cfg_map());
    // 18-point path with irregular spacing (mirrors the panicking shape).
    let route: Vec<(f32, f32)> = (0..18)
        .map(|i| (i as f32 * 0.37, (i % 3) as f32 * 0.11))
        .collect();
    let plan = solver::plan(&map, &route, (0.0, 0.0, 0.0), 1.0, None, &cfg);
    assert!(plan.poses.len() >= 2);
}

fn cfg_map() -> CostmapConfig {
    CostmapConfig::default()
}

#[test]
fn jittered_resolves_do_not_reverse_direction() {
    // hl61 forensics: the robot spun in circles on the wp4 leg, but 450
    // chained offline solves through that exact recorded window never flipped
    // direction — the spin was the FOLLOWER acting on backlogged (stale)
    // paths, not solver instability. This pins the solver half of that
    // finding: consecutive commitment-chained solves from jittered poses must
    // keep a consistent initial bearing.
    let cfg = SolverConfig::default();
    let map = costmap::build(&voxel_ground(8.0, 0.0), (0.0, 0.0, 0.4), 0.4, &cfg_map());
    let route = vec![(0.0, 0.0), (5.0, 0.0)];
    let first = solver::plan(&map, &route, (0.0, 0.0, 0.0), 1.0, None, &cfg);
    assert!(first.poses.len() >= 3);
    let mut prev: Vec<(f32, f32)> = first.poses.iter().map(|p| (p.0, p.1)).collect();
    let mut last_bearing = initial_bearing(&first.poses);
    for jitter in [-0.04f32, 0.03, -0.02, 0.05] {
        let plan = solver::plan(&map, &route, (0.02, jitter, 0.0), 1.0, Some(&prev), &cfg);
        assert!(plan.poses.len() >= 3);
        let b = initial_bearing(&plan.poses);
        let mut d = (b - last_bearing).abs();
        if d > std::f32::consts::PI {
            d = 2.0 * std::f32::consts::PI - d;
        }
        assert!(
            d < std::f32::consts::FRAC_PI_2,
            "direction reversed under pose jitter {jitter}: {last_bearing} -> {b}"
        );
        last_bearing = b;
        prev = plan.poses.iter().map(|p| (p.0, p.1)).collect();
    }
}

fn initial_bearing(poses: &[(f32, f32, f32)]) -> f32 {
    let k = poses.len().min(5) - 1;
    (poses[k].1 - poses[0].1).atan2(poses[k].0 - poses[0].0)
}

#[test]
fn level_reference_converges_after_a_latched_offset() {
    // hl78: the reference latched ~0.2 m high at a stairs base and the pure
    // hold-within-hysteresis kept it forever on flat ground, raising the slice
    // band enough to pull a doorway lintel into the costmap (start-blocked for
    // a whole leg). Within the hysteresis band the reference must CONVERGE to
    // the robot's z; a storey jump must still reset outright.
    let mut level = costmap::LevelTracker::default();
    assert_eq!(level.update(0.6, 0.25), 0.6); // first sample adopts
                                              // Flat ground at z=0.4 (offset 0.2 <= hysteresis): converges, not holds.
    let mut reference = 0.0;
    for _ in 0..24 {
        reference = level.update(0.4, 0.25);
    }
    assert!(
        (reference - 0.4).abs() < 0.02,
        "reference should converge to z on flat ground, still at {reference}"
    );
    // A storey jump resets immediately (unchanged semantics).
    assert_eq!(level.update(3.0, 0.25), 3.0);
}

/// A closed box sitting on the ground, sampled at voxel spacing like the
/// cleared map delivers it: shell only (top + faces), nothing inside.
fn box_shell(cx: f32, cy: f32, half: f32, height: f32) -> Vec<[f32; 3]> {
    let mut pts = Vec::new();
    let mut x = cx - half;
    while x <= cx + half {
        let mut y = cy - half;
        while y <= cy + half {
            pts.push([x, y, height]); // top
            y += 0.1;
        }
        x += 0.1;
    }
    let mut z = 0.1;
    while z < height {
        let mut t = -half;
        while t <= half {
            pts.push([cx - half, cy + t, z]);
            pts.push([cx + half, cy + t, z]);
            pts.push([cx + t, cy - half, z]);
            pts.push([cx + t, cy + half, z]);
            t += 0.1;
        }
        z += 0.1;
    }
    pts
}

fn lethal_fraction(map: &costmap::Costmap, x0: f32, x1: f32, y0: f32, y1: f32) -> f32 {
    let mut n = 0;
    let mut lethal = 0;
    let mut x = x0;
    while x <= x1 {
        let mut y = y0;
        while y <= y1 {
            if let Some((r, c)) = map.cell(x, y) {
                n += 1;
                if map.cost[map.index(r, c)] >= costmap::LETHAL_THRESHOLD {
                    lethal += 1;
                }
            }
            y += 0.1;
        }
        x += 0.1;
    }
    lethal as f32 / n.max(1) as f32
}

/// The failure Jeff reported on the 2026-07-09 warehouse recording: suitcases
/// (0.35-0.7 m — below can_climb) read FREE. The plateau-step gate must make
/// the box footprint lethal; without it the Sobel scores the 0.45 m edges ~37.
#[test]
fn sub_can_climb_box_is_lethal_with_plateau_gate() {
    let cfg = CostmapConfig::default();
    let mut pts = voxel_ground(8.0, 0.0);
    pts.extend(box_shell(2.0, 0.0, 0.3, 0.45));
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    // The gate guarantees the rim + one dilation ring; the very center of a
    // perfectly flat top has no lower reference within the 5x5 and can stay
    // free — unreachable anyway behind the lethal ring + solver inflation.
    // (Real suitcase tops are noisy and score higher — 78-85% on the
    // warehouse recording blobs.)
    let on_box = lethal_fraction(&map, 1.75, 2.25, -0.25, 0.25);
    assert!(
        on_box > 0.5,
        "box footprint must be majority-lethal, got {on_box}"
    );
    let rim = lethal_fraction(&map, 1.7, 2.3, -0.3, -0.2);
    assert!(rim > 0.85, "box rim must be lethal, got {rim}");
    let floor_near = lethal_fraction(&map, 0.0, 1.4, -1.0, 1.0);
    assert!(
        floor_near < 0.05,
        "floor near the box must stay free, got {floor_near}"
    );

    // Documents the pre-gate failure with the old config (can_climb 0.6,
    // no plateau gate — what the sim blueprint still runs): the gradient
    // alone leaves the sub-can_climb box free.
    let off = CostmapConfig {
        max_step: 0.0,
        can_climb: 0.6,
        ..CostmapConfig::default()
    };
    let map_off = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &off);
    let on_box_off = lethal_fraction(&map_off, 1.75, 2.25, -0.25, 0.25);
    assert!(
        on_box_off < 0.3,
        "documents the pre-gate failure: gradient alone leaves the box free (got {on_box_off})"
    );
}

/// dim_city-steep staircase (0.34 m risers per 0.1 m cell) with OPEN risers:
/// every third column of tread cells is a through-hole seeing the ground —
/// the terrain-map checkerboard that made naive step rules read treads as
/// isolated towers. Under the SIM blueprint config (the dim_city stairs are
/// steeper than the robot-physical defaults, so the blueprint overrides
/// max_grade to 6.0 and disables the gate) the flight must stay traversable.
#[test]
fn open_riser_steep_staircase_stays_free() {
    let cfg = CostmapConfig {
        can_climb: 0.6,
        max_step: 0.0,
        ..CostmapConfig::default()
    };
    let mut pts = voxel_ground(8.0, 0.0);
    let mut i = 0;
    let mut x = 1.0_f32;
    while x <= 3.0 {
        let tread_z = 3.4 * (x - 1.0) / 2.0; // 0 -> 3.4 over 2 m, dim_city grade
        if tread_z > 1.8 {
            break;
        }
        let mut y = -0.6;
        while y <= 0.6 {
            if i % 3 == 2 {
                pts.push([x, y, 0.0]); // through-hole: lidar sees the ground
            } else {
                pts.push([x, y, tread_z]);
            }
            y += 0.1;
        }
        i += 1;
        x += 0.1;
    }
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    let on_flight = lethal_fraction(&map, 1.1, 2.0, -0.5, 0.5);
    assert!(
        on_flight < 0.35,
        "steep open-riser flight must stay mostly free, got {on_flight}"
    );
    let approach = lethal_fraction(&map, 0.0, 0.9, -0.5, 0.5);
    assert!(
        approach < 0.05,
        "approach floor must stay free, got {approach}"
    );
}

/// Gentle solid stairs (0.17 m risers, 0.3 m treads — the real warehouse
/// staircase) must stay open with the gate enabled.
#[test]
fn gentle_solid_staircase_stays_free() {
    let cfg = CostmapConfig::default();
    let mut pts = voxel_ground(8.0, 0.0);
    let mut x = 1.0_f32;
    while x <= 3.4 {
        let step_i = ((x - 1.0) / 0.3).floor();
        let tread_z = 0.17 * (step_i + 1.0);
        let mut y = -0.6;
        while y <= 0.6 {
            pts.push([x, y, tread_z]);
            y += 0.1;
        }
        x += 0.1;
    }
    let map = costmap::build(&pts, (0.0, 0.0, 0.4), 0.4, &cfg);
    // Walking corridor only: the flight's open SIDES (tread surface a metre
    // above the ground beside it) are genuine drop-off edges and correctly
    // read lethal — same as the rail-edge band on the warehouse recording.
    let on_flight = lethal_fraction(&map, 1.15, 3.0, -0.3, 0.3);
    assert!(
        on_flight < 0.15,
        "gentle stairs corridor must stay open, got {on_flight}"
    );
}
