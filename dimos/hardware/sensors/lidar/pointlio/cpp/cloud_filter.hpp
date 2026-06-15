// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Point cloud filtering utilities: voxel grid downsampling and
// statistical outlier removal using PCL.

#ifndef CLOUD_FILTER_HPP_
#define CLOUD_FILTER_HPP_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CloudFilterConfig {
    float voxel_size = 0.1f;           // downsample leaf size (m); <=0 skips downsampling
    int outlier_neighbor_count = 50;   // statistical-outlier-removal neighbors; <=0 skips it
    float outlier_std_threshold = 1.0f;
};

/// Voxel-grid downsample + statistical outlier removal; each stage is skipped
/// when its config is <=0. Returns the filtered cloud (new allocation).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filter_cloud(
    const typename pcl::PointCloud<PointT>::Ptr& input,
    const CloudFilterConfig& cfg) {

    if (!input || input->empty()) { return input; }

    auto working = input;
    if (cfg.voxel_size > 0.0f) {
        typename pcl::PointCloud<PointT>::Ptr voxelized(new pcl::PointCloud<PointT>());
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(working);
        vg.setLeafSize(cfg.voxel_size, cfg.voxel_size, cfg.voxel_size);
        vg.filter(*voxelized);
        working = voxelized;
    }

    if (cfg.outlier_neighbor_count > 0 &&
        working->size() > static_cast<size_t>(cfg.outlier_neighbor_count)) {
        typename pcl::PointCloud<PointT>::Ptr cleaned(new pcl::PointCloud<PointT>());
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(working);
        sor.setMeanK(cfg.outlier_neighbor_count);
        sor.setStddevMulThresh(cfg.outlier_std_threshold);
        sor.filter(*cleaned);
        return cleaned;
    }

    return working;
}

#endif
