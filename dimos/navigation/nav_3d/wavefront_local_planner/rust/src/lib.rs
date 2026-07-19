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

//! Wavefront (navigation-function) local planner core.
//!
//! Pure algorithm crate: `costmap` builds the level-aware occupancy + chamfer
//! clearance field from raw points; `solver` runs the wavefront plan. Bindings
//! are feature-gated so the same core serves the dimos native module
//! ("native"), offline Python tests ("python"), and the browser demo ("wasm").

pub mod costmap;
pub mod solver;

#[cfg(feature = "python")]
mod python;

#[cfg(feature = "wasm")]
mod wasm;
