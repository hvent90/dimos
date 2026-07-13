// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! pyo3 bindings for offline parity tests against the Python reference planner.

use numpy::{PyArrayMethods, PyReadonlyArray2};
use pyo3::prelude::*;

use crate::costmap::{self, CostmapConfig};
use crate::solver::{self, SolverConfig};

/// Build a costmap from Nx3 points and plan; returns the (x, y, yaw) poses.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn plan_once(
    points: PyReadonlyArray2<f32>,
    robot: (f32, f32, f32),
    robot_z: f32,
    global_path: Vec<(f32, f32)>,
    speed: f32,
    resolution: f32,
) -> PyResult<Vec<(f32, f32, f32)>> {
    let pts: Vec<[f32; 3]> = points
        .as_array()
        .rows()
        .into_iter()
        .map(|r| [r[0], r[1], r[2]])
        .collect();
    let ccfg = CostmapConfig {
        resolution,
        ..CostmapConfig::default()
    };
    let scfg = SolverConfig::default();
    let map = costmap::build(&pts, (robot.0, robot.1, robot_z), robot_z, &ccfg);
    let plan = solver::plan(&map, &global_path, robot, speed, None, &scfg);
    Ok(plan.poses)
}

/// plan_once with a previous path for commitment/hysteresis chaining.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn plan_once_prev(
    points: PyReadonlyArray2<f32>,
    robot: (f32, f32, f32),
    robot_z: f32,
    global_path: Vec<(f32, f32)>,
    speed: f32,
    resolution: f32,
    previous: Vec<(f32, f32)>,
) -> PyResult<Vec<(f32, f32, f32)>> {
    let pts: Vec<[f32; 3]> = points
        .as_array()
        .rows()
        .into_iter()
        .map(|r| [r[0], r[1], r[2]])
        .collect();
    let ccfg = CostmapConfig {
        resolution,
        ..CostmapConfig::default()
    };
    let scfg = SolverConfig::default();
    let map = costmap::build(&pts, (robot.0, robot.1, robot_z), robot_z, &ccfg);
    let prev_opt = (previous.len() >= 2).then_some(previous.as_slice());
    let plan = solver::plan(&map, &global_path, robot, speed, prev_opt, &scfg);
    Ok(plan.poses)
}

/// Build the internal costmap alone and return (cost HxW i8, (origin_x, origin_y), resolution).
///
/// The knobs mirror CostmapConfig so offline tuning sweeps them from Python
/// without a rebuild; defaults match the shipped module config.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[pyo3(signature = (points, robot, reference_z, resolution=0.1, can_pass_under=0.6,
                    max_grade=3.0, max_safe_fall=0.5, void_depth_lethal=2.5,
                    slice_below=1.1, slice_above=1.5, half_extent=8.0, level_hysteresis=0.25,
                    body_step=0.35, body_min_points=0, body_min_extent=0.1, max_step=0.3))]
fn build_costmap<'py>(
    py: Python<'py>,
    points: PyReadonlyArray2<f32>,
    robot: (f32, f32, f32),
    reference_z: f32,
    resolution: f32,
    can_pass_under: f32,
    max_grade: f32,
    max_safe_fall: f32,
    void_depth_lethal: f32,
    slice_below: f32,
    slice_above: f32,
    half_extent: f32,
    level_hysteresis: f32,
    body_step: f32,
    body_min_points: u16,
    body_min_extent: f32,
    max_step: f32,
) -> PyResult<(Bound<'py, numpy::PyArray2<i8>>, (f32, f32), f32)> {
    let pts: Vec<[f32; 3]> = points
        .as_array()
        .rows()
        .into_iter()
        .map(|r| [r[0], r[1], r[2]])
        .collect();
    let ccfg = CostmapConfig {
        resolution,
        can_pass_under,
        can_climb: max_grade * resolution,
        max_safe_fall,
        void_depth_lethal,
        slice_below,
        slice_above,
        half_extent,
        level_hysteresis,
        body_step,
        body_min_points,
        body_min_extent,
        max_step,
        ..CostmapConfig::default()
    };
    let map = costmap::build(&pts, robot, reference_z, &ccfg);
    let arr = numpy::PyArray1::from_vec(py, map.cost).reshape([map.height, map.width])?;
    Ok((arr, map.origin, map.resolution))
}

#[pymodule]
fn dimos_repulsive_field(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(plan_once, m)?)?;
    m.add_function(wrap_pyfunction!(plan_once_prev, m)?)?;
    m.add_function(wrap_pyfunction!(build_costmap, m)?)?;
    Ok(())
}
