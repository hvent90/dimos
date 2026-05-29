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

// Read the Go2 MCAP directly and re-emit it as the PLNR1 byte stream the rest
// of laserMapping.cpp already knows how to parse. This is the C++ port of
// human-debug/mcap_to_plnr1.py: same topics, same record layout, lidar blob
// copied verbatim, ts = mcap publish_time. Verified byte-identical to the
// committed go2-185959.bin.

// Only zstd chunks are present in our recordings; skip the lz4 backend so we
// don't need liblz4. The mcap reader implementation is compiled here (and only
// here) via MCAP_IMPLEMENTATION.
#define MCAP_COMPRESSION_NO_LZ4
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include "mcap_source.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr std::string_view TOPIC_CLOUD = "rt/utlidar/cloud";
constexpr std::string_view TOPIC_IMU = "rt/utlidar/imu";

// Minimal little-endian CDR (XCDR1) cursor — the C++ twin of go2_cdr.Cur.
// Alignment is body-relative: the 4-byte encapsulation header is skipped, and
// each primitive aligns to its size measured from there.
struct Cur {
  const uint8_t* b;
  size_t n;
  size_t p = 4;  // past the 4-byte CDR encapsulation header

  void align(size_t a) {
    size_t m = (p - 4) % a;
    if (m) p += a - m;
  }
  uint8_t u8() { return b[p++]; }
  uint32_t u32() {
    align(4);
    uint32_t v;
    std::memcpy(&v, b + p, 4);
    p += 4;
    return v;
  }
  int32_t i32() {
    align(4);
    int32_t v;
    std::memcpy(&v, b + p, 4);
    p += 4;
    return v;
  }
  double f64() {
    align(8);
    double v;
    std::memcpy(&v, b + p, 8);
    p += 8;
    return v;
  }
  void skip_str() { p += u32(); }  // length-prefixed; advance past the bytes
  void skip_f64(int k) {
    for (int i = 0; i < k; i++) f64();
  }
};

// PLNR1 record header: type(1) + 7 pad + ts_sec(double). ts is the mcap
// publish_time (device stamp), matching the Python converter.
void append_header(std::string& s, uint8_t type, uint64_t ts_ns) {
  const char pad[7] = {0};
  s.append(reinterpret_cast<const char*>(&type), 1);
  s.append(pad, 7);
  double ts = static_cast<double>(ts_ns) / 1e9;
  s.append(reinterpret_cast<const char*>(&ts), 8);
}

}  // namespace

std::string read_mcap_as_plnr1(const std::string& mcap_path) {
  mcap::McapReader reader;
  const mcap::Status st = reader.open(mcap_path);
  if (!st.ok()) {
    throw std::runtime_error("mcap open failed for " + mcap_path + ": " + st.message);
  }

  mcap::ReadMessageOptions opts;
  opts.topicFilter = [](std::string_view t) { return t == TOPIC_CLOUD || t == TOPIC_IMU; };

  std::vector<std::pair<uint64_t, std::string>> recs;  // (ts_ns, PLNR1 payload)
  const auto on_problem = [](const mcap::Status&) {};

  for (const auto& mv : reader.readMessages(on_problem, opts)) {
    const auto* data = reinterpret_cast<const uint8_t*>(mv.message.data);
    const size_t size = mv.message.dataSize;
    const uint64_t ts = mv.message.publishTime;
    const std::string& topic = mv.channel->topic;

    std::string rec;
    if (topic == TOPIC_IMU) {
      Cur c{data, size};
      c.i32();
      c.u32();         // header.stamp (sec, nsec) — unused; ts is publish_time
      c.skip_str();    // header.frame_id
      c.skip_f64(4);   // orientation (x,y,z,w)
      c.skip_f64(9);   // orientation_covariance
      double gx = c.f64(), gy = c.f64(), gz = c.f64();  // angular_velocity
      c.skip_f64(9);                                    // angular_velocity_covariance
      double ax = c.f64(), ay = c.f64(), az = c.f64();  // linear_acceleration
      append_header(rec, 1, ts);
      const double v[6] = {gx, gy, gz, ax, ay, az};
      rec.append(reinterpret_cast<const char*>(v), sizeof(v));
    } else {
      Cur c{data, size};
      c.i32();
      c.u32();      // header.stamp
      c.skip_str();  // header.frame_id
      c.u32();       // height
      c.u32();       // width
      uint32_t nf = c.u32();
      for (uint32_t i = 0; i < nf; i++) {
        c.skip_str();  // field name
        c.u32();       // offset
        c.u8();        // datatype
        c.u32();       // count
      }
      c.u8();                // is_bigendian
      uint32_t ps = c.u32();  // point_step
      c.u32();                // row_step
      uint32_t nd = c.u32();  // data byte length
      if (ps != 32 || nd < 32) continue;  // matches converter: skip empty/odd clouds
      const uint8_t* blob = data + c.p;
      uint32_t npts = nd / ps;
      append_header(rec, 0, ts);
      uint32_t zero = 0;
      rec.append(reinterpret_cast<const char*>(&npts), 4);
      rec.append(reinterpret_cast<const char*>(&zero), 4);
      rec.append(reinterpret_cast<const char*>(blob), nd);  // verbatim L1 points
    }
    recs.emplace_back(ts, std::move(rec));
  }
  reader.close();

  // Time-sort (stable, like the Python converter) so the merged lidar+imu
  // stream is monotonic in publish_time.
  std::stable_sort(recs.begin(), recs.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });

  static constexpr char MAGIC[16] = {'P', 'L', 'N', 'R', '1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::string out;
  out.append(MAGIC, 16);
  for (const auto& r : recs) out += r.second;
  return out;
}
