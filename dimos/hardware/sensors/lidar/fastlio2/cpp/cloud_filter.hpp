// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Point cloud filtering utilities: voxel grid downsampling and
// statistical outlier removal using PCL.

#ifndef CLOUD_FILTER_HPP_
#define CLOUD_FILTER_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CloudFilterConfig {
    float voxel_size = 0.1f;
    int sor_mean_k = 50;
    float sor_stddev = 1.0f;
};

// Drop-in voxel-grid downsampler with int64 indices. Hash-based, so only
// occupied voxels are stored — works for arbitrarily large coordinate
// extents. pcl::VoxelGrid uses int32 for the linear voxel index and
// overflows when (max-min)/leaf exceeds ~2^31 (~129 m cube at 0.1 m).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxel_downsample_i64(
    const typename pcl::PointCloud<PointT>::Ptr& input, float leaf) {

    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>());
    if (!input || input->empty() || leaf <= 0.0f) {
        return out;
    }

    struct Key {
        int64_t x, y, z;
        bool operator==(const Key& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            size_t h = static_cast<size_t>(k.x) * 73856093ull;
            h ^= static_cast<size_t>(k.y) * 19349669ull;
            h ^= static_cast<size_t>(k.z) * 83492791ull;
            return h;
        }
    };
    struct Acc {
        double x = 0, y = 0, z = 0, intensity = 0;
        uint32_t count = 0;
        PointT first;  // preserves non-averaged fields (normals, curvature, ...)
    };

    std::unordered_map<Key, Acc, KeyHash> bins;
    bins.reserve(input->size() / 4 + 1);

    const double inv = 1.0 / static_cast<double>(leaf);
    for (const auto& pt : input->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        Key k{
            static_cast<int64_t>(std::floor(static_cast<double>(pt.x) * inv)),
            static_cast<int64_t>(std::floor(static_cast<double>(pt.y) * inv)),
            static_cast<int64_t>(std::floor(static_cast<double>(pt.z) * inv))};
        auto [it, inserted] = bins.try_emplace(k);
        auto& a = it->second;
        if (inserted) {
            a.first = pt;
        }
        a.x += pt.x;
        a.y += pt.y;
        a.z += pt.z;
        a.intensity += pt.intensity;
        a.count++;
    }

    out->points.reserve(bins.size());
    for (const auto& [k, a] : bins) {
        const double n = static_cast<double>(a.count);
        PointT pt = a.first;
        pt.x = static_cast<float>(a.x / n);
        pt.y = static_cast<float>(a.y / n);
        pt.z = static_cast<float>(a.z / n);
        pt.intensity = static_cast<float>(a.intensity / n);
        out->points.push_back(pt);
    }
    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = true;
    return out;
}

/// Apply voxel grid downsample + statistical outlier removal in-place.
/// Returns the filtered cloud (new allocation).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filter_cloud(
    const typename pcl::PointCloud<PointT>::Ptr& input,
    const CloudFilterConfig& cfg) {

    if (!input || input->empty()) return input;

    // Voxel grid downsample (int64-indexed, hash-based — no overflow)
    auto voxelized = voxel_downsample_i64<PointT>(input, cfg.voxel_size);

    // Statistical outlier removal
    if (cfg.sor_mean_k > 0 && voxelized->size() > static_cast<size_t>(cfg.sor_mean_k)) {
        typename pcl::PointCloud<PointT>::Ptr cleaned(new pcl::PointCloud<PointT>());
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(voxelized);
        sor.setMeanK(cfg.sor_mean_k);
        sor.setStddevMulThresh(cfg.sor_stddev);
        sor.filter(*cleaned);
        return cleaned;
    }

    return voxelized;
}

#endif
