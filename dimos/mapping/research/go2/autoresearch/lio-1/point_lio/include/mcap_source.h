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

#pragma once

#include <string>

// Read a Go2 MCAP recording (rt/utlidar/cloud + rt/utlidar/imu) and return the
// exact PLNR1 byte stream that mcap_to_plnr1.py would have written — magic
// header + time-sorted lidar/imu records, lidar point blobs verbatim. The
// returned buffer is meant to be handed to fmemopen() so the existing PLNR1
// reader in laserMapping.cpp consumes it unchanged.
//
// Throws std::runtime_error if the file can't be opened/parsed.
std::string read_mcap_as_plnr1(const std::string& mcap_path);
