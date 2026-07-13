#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <okvis_pose_graph_msgs/msg/pose_graph.hpp>

#include "edge_hessian.hpp"

namespace ORB_SLAM3 {
class System;
class KeyFrame;
class Map;
}  // namespace ORB_SLAM3

namespace orb_slam3_ros2 {

// Builds the okvis_pose_graph_msgs/PoseGraph snapshot from the current active
// map of an ORB-SLAM3 System, following the z-floc pose-graph contract:
//   - vertex_pose = KeyFrame body pose T_WS (GetImuPose), sorted by mnId
//   - VO edges (type 0) follow the mPrevKF chain (robust to KF culling)
//   - loop edges (type 1) from GetLoopEdges() + GetMergeEdges()
//   - per-edge info = reprojection-Hessian over co-observed MapPoints
// Edge info is memoized per (id_a,id_b) and invalidated once when the IMU is
// initialized (poses/depths get rescaled). edge_rel and vertex poses are always
// recomputed from the current geometry.
class PoseGraphBuilder {
 public:
  // stamp_ns maps an ORB-SLAM3 double-seconds timestamp to the exact ns stamp
  // of the originating image (falls back to llround(t*1e9) if unknown).
  using StampFn = std::function<int64_t(double)>;

  explicit PoseGraphBuilder(ORB_SLAM3::System* sys) : sys_(sys) {}

  // Returns the full incremental snapshot. multi_map_out is set true when the
  // atlas holds more than one map (only the active map is published).
  okvis_pose_graph_msgs::msg::PoseGraph Build(const StampFn& stamp_ns,
                                              const rclcpp::Time& header_stamp,
                                              bool* multi_map_out);

 private:
  EdgeInfo EdgeInfoFor(ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b);

  ORB_SLAM3::System* sys_;
  std::map<std::pair<uint64_t, uint64_t>, EdgeInfo> info_cache_;
  bool imu_init_seen_ = false;
};

}  // namespace orb_slam3_ros2
