#pragma once
#include "commons.h"
#include "scan_context.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_time_thresh = 60.0;
    double loop_score_thresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;
    // Sanity gate: skip ICP if candidate keyframe is farther than this
    // from current keyframe in global pose. 0 disables the check.
    double loop_candidate_max_distance_m = 30.0;

    // Feature-poverty gate: skip loop search when the current scan's
    // descriptor vertical-structure std (scan_context::descriptor_structure)
    // is below this — the scan can't reliably place itself (open grass field),
    // so any closure would be noise. 0 disables (default = current behavior).
    // Superseded in practice by loop_min_occupancy + loop_min_degeneracy below
    // (structure overlaps too much between scenes to threshold cleanly).
    double min_descriptor_std = 0.0;

    // Structure-spread gate: require at least this many occupied Scan-Context
    // cells. Open grass returns cluster near the sensor (few rings filled);
    // built environments spread returns out to range. Calibrated on go2 fastlio
    // (1200-cell 20x60 descriptor): grassy ~70 vs gir_park ~88 vs downtown ~120
    // at the SAME point count, so this measures spread, not density. 0 disables.
    int loop_min_occupancy = 80;

    // Observability gate (Zhang 2016 / X-ICP degeneracy factor): reject a loop
    // candidate whose source scan's smallest normalized normal-scatter
    // eigenvalue is below this. A planar/degenerate scan (open grass) -> ~0:
    // ICP slides freely in-plane and reports low fitness for a bogus closure.
    // Real scenes (incl. sparse gir_park) sit >0.15; grassy's firing closures
    // sit ~0.01. 0 disables.
    double loop_min_degeneracy = 0.05;

    // When true, log one "PGO_DIAG ..." line per loop candidate that reaches
    // ICP (accepted OR rejected) with its fitness / candidate distance /
    // descriptor structure — the data to design the right loop-acceptance gate.
    bool debug = false;

    // Scan Context settings
    bool use_scan_context = true;
    int scan_context_num_rings = 20;
    int scan_context_num_sectors = 60;
    double scan_context_max_range_m = 80.0;
    int scan_context_top_k = 10;
    double scan_context_match_threshold = 0.4;
    double scan_context_lidar_height_m = 2.0;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

    // Place recognition exposed for diagnostics / persistence.
    const std::vector<scan_context::Descriptor>& descriptors() const { return m_scan_context_descriptors; }
    const std::vector<scan_context::RingKey>& ringKeys() const { return m_scan_context_ring_keys; }

private:
    // Scan-context-based candidate search; returns -1 if no acceptable match.
    // out_best / out_second report the closest and 2nd-closest candidate cosine
    // distances (for the Lowe ratio distinctiveness signal).
    int searchByScanContext(int& out_sector_shift, float& out_best, float& out_second) const;
    // Original position-based fallback (radius search on past key-pose
    // positions). Kept for ablation + when scan context is disabled.
    int searchByPosition() const;

    Config m_config;
    scan_context::Config m_scan_context_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    std::vector<scan_context::Descriptor> m_scan_context_descriptors;
    std::vector<scan_context::RingKey> m_scan_context_ring_keys;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;
};
