#include "pose_graph_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sophus/se3.hpp>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace orb_slam3_ros2 {

namespace {

geometry_msgs::msg::Pose ToPoseMsg(const Sophus::SE3f& T) {
  const Eigen::Quaternionf q = T.unit_quaternion();
  const Eigen::Vector3f t = T.translation();
  geometry_msgs::msg::Pose p;
  p.position.x = t.x();
  p.position.y = t.y();
  p.position.z = t.z();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

}  // namespace

EdgeInfo PoseGraphBuilder::EdgeInfoFor(ORB_SLAM3::KeyFrame* a,
                                       ORB_SLAM3::KeyFrame* b) {
  // Shared MapPoints (co-observed landmarks).
  std::unordered_set<ORB_SLAM3::MapPoint*> setA;
  for (ORB_SLAM3::MapPoint* mp : a->GetMapPointMatches()) {
    if (mp && !mp->isBad()) setA.insert(mp);
  }
  std::vector<Eigen::Vector3d> pts_w;
  pts_w.reserve(setA.size());
  for (ORB_SLAM3::MapPoint* mp : b->GetMapPointMatches()) {
    if (mp && !mp->isBad() && setA.count(mp))
      pts_w.push_back(mp->GetWorldPos().cast<double>());
  }
  if (pts_w.empty()) return EdgeInfo{};

  const Sophus::SE3f TwcA = a->GetPoseInverse();
  const Sophus::SE3f TwcB = b->GetPoseInverse();
  const Eigen::Matrix3d R_wc_a = TwcA.rotationMatrix().cast<double>();
  const Eigen::Vector3d t_wc_a = TwcA.translation().cast<double>();
  const Eigen::Matrix3d R_wc_b = TwcB.rotationMatrix().cast<double>();
  const Eigen::Vector3d t_wc_b = TwcB.translation().cast<double>();

  const double focal = std::max(static_cast<double>(a->fx),
                                static_cast<double>(a->fy));
  const double cols = static_cast<double>(a->mnMaxX);
  const double rows = static_cast<double>(a->mnMaxY);
  return ComputeEdgeHessianInfo(pts_w, R_wc_a, t_wc_a, R_wc_b, t_wc_b,
                                focal, cols, rows, /*sigma_px=*/1.0);
}

okvis_pose_graph_msgs::msg::PoseGraph PoseGraphBuilder::Build(
    const StampFn& stamp_ns, const rclcpp::Time& header_stamp,
    bool* multi_map_out) {
  okvis_pose_graph_msgs::msg::PoseGraph msg;
  msg.header.stamp = header_stamp;
  msg.header.frame_id = "world";

  ORB_SLAM3::Atlas* atlas = sys_->GetAtlas();
  if (!atlas) return msg;
  ORB_SLAM3::Map* pMap = atlas->GetCurrentMap();
  if (!pMap) return msg;
  if (multi_map_out) *multi_map_out = (atlas->CountMaps() > 1);

  // Snapshot under the same mutex LocalMapping / LoopClosing hold while
  // rewriting poses, so vertices and edges are mutually consistent.
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  if (pMap->isImuInitialized() && !imu_init_seen_) {
    info_cache_.clear();  // depths rescaled at IMU init -> recompute infos
    imu_init_seen_ = true;
  }

  std::vector<ORB_SLAM3::KeyFrame*> kfs = pMap->GetAllKeyFrames();
  kfs.erase(std::remove_if(kfs.begin(), kfs.end(),
                           [](ORB_SLAM3::KeyFrame* kf) {
                             return !kf || kf->isBad();
                           }),
            kfs.end());
  std::sort(kfs.begin(), kfs.end(), ORB_SLAM3::KeyFrame::lId);
  if (kfs.empty()) return msg;

  // Vertices + per-id lookups.
  std::unordered_map<uint64_t, ORB_SLAM3::KeyFrame*> id2kf;
  std::unordered_map<uint64_t, Sophus::SE3f> id2Twb;  // body world pose
  id2kf.reserve(kfs.size());
  id2Twb.reserve(kfs.size());
  for (ORB_SLAM3::KeyFrame* kf : kfs) {
    const uint64_t id = kf->mnId;
    const Sophus::SE3f Twb = kf->GetImuPose();  // T_WS (world <- sensor/body)
    msg.vertex_id.push_back(id);
    msg.vertex_stamp_ns.push_back(stamp_ns(kf->mTimeStamp));
    msg.vertex_pose.push_back(ToPoseMsg(Twb));
    id2kf[id] = kf;
    id2Twb[id] = Twb;
  }

  auto add_edge = [&](ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b,
                      uint8_t type) {
    const uint64_t ia = a->mnId, ib = b->mnId;
    const Sophus::SE3f& Twa = id2Twb[ia];
    const Sophus::SE3f& Twb = id2Twb[ib];
    const Sophus::SE3f Tab = Twa.inverse() * Twb;  // T_AB = T_WA^-1 * T_WB

    // Memoized reprojection-Hessian info; recompute only if not cached with a
    // usable (>=8 landmark) result yet.
    const std::pair<uint64_t, uint64_t> key{ia, ib};
    auto it = info_cache_.find(key);
    if (it == info_cache_.end() || it->second.n_used < 8) {
      EdgeInfo ei = EdgeInfoFor(a, b);
      it = info_cache_.insert_or_assign(key, ei).first;
    }
    const EdgeInfo& ei = it->second;
    const double info = (ei.n_used >= 8) ? ei.info_trans : 1.0;
    const double info_r = (ei.n_used >= 8) ? ei.info_rot : 1.0;

    msg.edge_i.push_back(ia);
    msg.edge_j.push_back(ib);
    msg.edge_rel.push_back(ToPoseMsg(Tab));
    msg.edge_info_trans.push_back(info);
    msg.edge_info_rot.push_back(info_r);
    msg.edge_type.push_back(type);
  };

  // VO (sequential) edges: follow the inertial mPrevKF chain.
  for (ORB_SLAM3::KeyFrame* kf : kfs) {
    ORB_SLAM3::KeyFrame* prev = kf->mPrevKF;
    if (!prev || prev->isBad()) continue;
    if (!id2Twb.count(prev->mnId)) continue;  // prev not in this snapshot
    add_edge(prev, kf, /*type=*/0);
  }

  // Loop / merge edges (type 1), deduplicated as ordered pairs.
  std::set<std::pair<uint64_t, uint64_t>> loop_seen;
  auto add_loops = [&](ORB_SLAM3::KeyFrame* kf,
                       const std::set<ORB_SLAM3::KeyFrame*>& edges) {
    for (ORB_SLAM3::KeyFrame* lkf : edges) {
      if (!lkf || lkf->isBad()) continue;
      if (!id2Twb.count(lkf->mnId)) continue;
      const uint64_t a = std::min(kf->mnId, lkf->mnId);
      const uint64_t b = std::max(kf->mnId, lkf->mnId);
      if (a == b) continue;
      if (!loop_seen.insert({a, b}).second) continue;
      add_edge(id2kf[a], id2kf[b], /*type=*/1);
    }
  };
  for (ORB_SLAM3::KeyFrame* kf : kfs) {
    add_loops(kf, kf->GetLoopEdges());
    add_loops(kf, kf->GetMergeEdges());
  }

  return msg;
}

}  // namespace orb_slam3_ros2
