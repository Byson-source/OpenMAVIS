// z-floc mono-inertial OpenMAVIS (ORB-SLAM3 fork) front-end.
//
// Subscribes cam0 (compressed) + IMU, drives official ORB-SLAM3 in IMU_MONOCULAR
// mode, and publishes the five topics the zfloc e2e pipeline consumes under the
// /openmavis namespace:
//   <ns>/odometry        nav_msgs/Odometry           per-frame camera pose (T_WC)
//   <ns>/trajectory      nav_msgs/Path               per-KF snapshot (T_WC)
//   <ns>/trajectory_final nav_msgs/Path              dense loop-corrected frames (T_WC), at shutdown
//   <ns>/image/compressed sensor_msgs/CompressedImage KF image (re-emitted)
//   <ns>/pose_graph      okvis_pose_graph_msgs/PoseGraph incremental snapshot per KF
//
// Tracking runs on a dedicated worker thread so the ROS executor never starves
// the 1 kHz IMU intake. Pose-graph vertices are body poses (T_WS); trajectory /
// odometry are camera poses (T_WC), matching the OV-SLAM front-end conventions.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <okvis_pose_graph_msgs/msg/pose_graph.hpp>

#include <sophus/se3.hpp>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "ImuTypes.h"
#include "Tracking.h"

#include "pose_graph_publisher.hpp"

namespace {
std::atomic<bool> g_stop{false};
void OnSigint(int) { g_stop.store(true); }

int64_t NsFromSec(double t_sec) { return std::llround(t_sec * 1e9); }
uint64_t DoubleBits(double d) {
  uint64_t b;
  std::memcpy(&b, &d, sizeof(b));
  return b;
}

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
rclcpp::Time StampFromNs(int64_t ns) {
  return rclcpp::Time(ns, RCL_ROS_TIME);
}
}  // namespace

using orb_slam3_ros2::PoseGraphBuilder;

class OrbSlam3Node : public rclcpp::Node {
 public:
  OrbSlam3Node()
      : Node("hilti_mono_inertial_node") {
    voc_file_ = declare_parameter<std::string>("voc_file", "");
    settings_file_ = declare_parameter<std::string>("settings_file", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/cam0/image_raw/compressed");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    output_ns_ = declare_parameter<std::string>("output_ns", "/openmavis");
    // cam-IMU timeshift [s]: t_cam_corrected = t_cam - image_delay (matches OKVIS2-X
    // hilti_challenge_2026 image_delay). Fed to ORB tracking / IMU alignment only;
    // published stamps stay on the original bag timeline.
    image_delay_ = declare_parameter<double>("image_delay", 0.006569);
    if (!output_ns_.empty() && output_ns_.back() == '/')
      output_ns_.pop_back();

    // Fisheye -> pinhole undistort. The Hilti cam0 is a ~200 deg fisheye; the raw
    // frame as KannalaBrandt8 makes ORB tracking collapse. We rectify each frame to
    // a virtual pinhole (R = I, so the optical frame / IMU.T_b_c1 is unchanged) and
    // feed THAT to ORB. The settings_file must be the matching PinHole config. The
    // re-emitted /image topic stays the RAW fisheye (downstream reconstructs from the
    // raw panorama bag, and poses are in the same cam0 frame). Defaults = proven
    // DROID-W recipe (project_droidw_hilti_fisheye_undistort) + calib.txt Knew.
    // Default false: feed the RAW fisheye and let ORB's KannalaBrandt8 model handle
    // distortion (matches the working ORB-SLAM3 setup; full FoV anchors the IMU). The
    // undistort->narrow-pinhole path let the inertial estimate diverge. Set undistort:=true
    // + a Rectified/PinHole config only to revisit the undistort experiment.
    undistort_ = declare_parameter<bool>("undistort", false);
    const double fx = declare_parameter<double>("fisheye_fx", 465.3015482593691);
    const double fy = declare_parameter<double>("fisheye_fy", 465.32303798346413);
    const double cx = declare_parameter<double>("fisheye_cx", 730.0455886686005);
    const double cy = declare_parameter<double>("fisheye_cy", 720.1427007671206);
    const double k1 = declare_parameter<double>("fisheye_k1", 0.025800718903376804);
    const double k2 = declare_parameter<double>("fisheye_k2", -0.010909240777406872);
    const double k3 = declare_parameter<double>("fisheye_k3", -0.0016899537986031076);
    const double k4 = declare_parameter<double>("fisheye_k4", 0.00014766801645260894);
    const double nfx = declare_parameter<double>("pinhole_fx", 700.0);
    const double nfy = declare_parameter<double>("pinhole_fy", 700.0);
    const double ncx = declare_parameter<double>("pinhole_cx", 736.0);
    const double ncy = declare_parameter<double>("pinhole_cy", 720.0);
    und_w_ = declare_parameter<int>("undistort_width", 1472);
    und_h_ = declare_parameter<int>("undistort_height", 1440);
    fish_K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    fish_D_ = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);
    new_K_ = (cv::Mat_<double>(3, 3) << nfx, 0, ncx, 0, nfy, ncy, 0, 0, 1);

    slam_ = std::make_unique<ORB_SLAM3::System>(
        voc_file_, settings_file_, ORB_SLAM3::System::IMU_MONOCULAR,
        /*bUseViewer=*/false);
    pgb_ = std::make_unique<PoseGraphBuilder>(slam_.get());

    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto qos_stream = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(output_ns_ + "/odometry", qos_stream);
    pub_traj_ = create_publisher<nav_msgs::msg::Path>(output_ns_ + "/trajectory", qos_reliable);
    pub_traj_final_ = create_publisher<nav_msgs::msg::Path>(output_ns_ + "/trajectory_final", qos_reliable);
    pub_image_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_ns_ + "/image/compressed", qos_stream);
    pub_graph_ = create_publisher<okvis_pose_graph_msgs::msg::PoseGraph>(output_ns_ + "/pose_graph", qos_reliable);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OrbSlam3Node::ImuCb, this, std::placeholders::_1));
    sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OrbSlam3Node::ImageCb, this, std::placeholders::_1));

    running_.store(true);
    worker_ = std::thread(&OrbSlam3Node::WorkerLoop, this);

    // record_openmavis_euler.sh greps for this exact line to know we are ready.
    RCLCPP_INFO(get_logger(), "subscribing image=%s imu=%s (output_ns=%s)",
                image_topic_.c_str(), imu_topic_.c_str(), output_ns_.c_str());
  }

  // Recover the dense trajectory, publish final snapshots, wait for the recorder
  // to ack them, and ONLY THEN shut SLAM down. Called from main after SIGINT.
  //
  // Ordering is deliberate: OpenMAVIS (like ORB-SLAM3) can segfault inside the
  // LoopClosing FUSE / final global-BA that System::Shutdown() runs on a
  // mono-inertial map. If Shutdown() crashes, everything after it is lost -> the
  // recorder never gets trajectory_final or the loop-closed pose graph and eval
  // falls back to the stale-scale /odometry stream (up-to-scale pre-VIBA poses).
  // So we flush the FINAL runtime map (already VIBA-metric + loop-corrected)
  // BEFORE Shutdown(); the final GBA is only a marginal refinement. Shutdown()
  // is then best-effort -- if it segfaults, the good data is already recorded.
  void Finalize() {
    running_.store(false);
    img_cv_.notify_all();
    imu_cv_.notify_all();
    if (worker_.joinable()) worker_.join();

    // Stop the loop closer from starting NEW fusions (the crash site); let any
    // in-flight loop fusion settle so the map is in a consistent state to read.
    RCLCPP_INFO(get_logger(), "deactivating loop closing; settling before flush...");
    slam_->DeActivateLC();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Flush the final map to the recorder FIRST (before the crash-prone Shutdown).
    PublishTrajectoryFinal();
    PublishTrajectorySnapshot();
    PublishPoseGraph();

    // HI-SLAM2 lesson: make sure the reliable end-of-run messages are acked by
    // the recorder before we risk the shutdown crash (it stays alive until this
    // node dies).
    pub_traj_final_->wait_for_all_acked(std::chrono::seconds(3));
    pub_graph_->wait_for_all_acked(std::chrono::seconds(3));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(get_logger(), "trajectory_final + pose_graph flushed; shutting down SLAM (best-effort)...");

    // Best-effort final shutdown (runs final GBA; may segfault -- data is safe).
    slam_->Shutdown();
    RCLCPP_INFO(get_logger(), "finalize done");
  }

 private:
  int64_t StampNs(double t_sec) {
    std::lock_guard<std::mutex> lk(stamp_mtx_);
    auto it = stamp_map_.find(DoubleBits(t_sec));
    return (it != stamp_map_.end()) ? it->second : NsFromSec(t_sec);
  }

  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    ORB_SLAM3::IMU::Point p(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, t);
    {
      std::lock_guard<std::mutex> lk(imu_mtx_);
      imu_buf_.push_back(p);
    }
    imu_cv_.notify_one();  // wake a worker waiting for IMU to reach an image time
  }

  void ImageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      img_queue_.push_back(msg);
    }
    img_cv_.notify_one();
  }

  void WorkerLoop() {
    while (running_.load()) {
      sensor_msgs::msg::CompressedImage::SharedPtr msg;
      {
        std::unique_lock<std::mutex> lk(img_mtx_);
        img_cv_.wait(lk, [&] { return !img_queue_.empty() || !running_.load(); });
        if (!running_.load() && img_queue_.empty()) break;
        msg = img_queue_.front();
        img_queue_.pop_front();
      }
      ProcessImage(msg);
    }
  }

  void ProcessImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg) {
    const int64_t ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    // Timeshift-corrected camera time handed to ORB (aligns cam with the IMU clock);
    // stamp_map_ maps it back to the original bag ns for all published stamps.
    const double t = ns * 1e-9 - image_delay_;
    // Map the corrected time (== KeyFrame::mTimeStamp inside ORB) back to the
    // original bag ns, so every published stamp stays on the bag timeline.
    {
      std::lock_guard<std::mutex> lk(stamp_mtx_);
      stamp_map_[DoubleBits(t)] = ns;
    }

    cv::Mat im = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (im.empty()) {
      RCLCPP_WARN(get_logger(), "failed to decode image @ %.6f", t);
      return;
    }

    // Rectify fisheye -> virtual pinhole before tracking (see ctor). Build the
    // remap tables once from the first frame's size (equidistant/KB8 model).
    if (undistort_) {
      if (!maps_ready_) {
        cv::fisheye::initUndistortRectifyMap(
            fish_K_, fish_D_, cv::Mat::eye(3, 3, CV_64F), new_K_,
            cv::Size(und_w_, und_h_), CV_16SC2, und_map1_, und_map2_);
        maps_ready_ = true;
        RCLCPP_INFO(get_logger(), "fisheye->pinhole undistort maps ready (%dx%d)", und_w_, und_h_);
      }
      cv::Mat rect;
      cv::remap(im, rect, und_map1_, und_map2_, cv::INTER_LINEAR);
      im = rect;
    }

    // Drain IMU samples up to this image time. CRUCIAL for mono-inertial: wait
    // until the IMU stream has caught up PAST the image time before draining, so
    // vImu covers the whole [prev_frame, this_frame] interval. Without this the
    // image callback can outrun the (separate) IMU callback and hand ORB a short
    // vImu, starving IMU initialization -> endless map resets. Times out near the
    // end of the bag (no more IMU will arrive).
    std::vector<ORB_SLAM3::IMU::Point> vimu;
    {
      std::unique_lock<std::mutex> lk(imu_mtx_);
      imu_cv_.wait_for(lk, std::chrono::seconds(2), [&] {
        return (!imu_buf_.empty() && imu_buf_.back().t >= t) || !running_.load();
      });
      while (!imu_buf_.empty() && imu_buf_.front().t <= t) {
        vimu.push_back(imu_buf_.front());
        imu_buf_.pop_front();
      }
    }

    const Sophus::SE3f Tcw = slam_->TrackMonocular(im, t, vimu);

    // Keep the original compressed image around for KF re-emission.
    ring_[ns] = msg;
    ring_order_.push_back(ns);
    while (ring_order_.size() > kRingCap) {
      ring_.erase(ring_order_.front());
      ring_order_.pop_front();
    }

    if (slam_->GetTracker()->mState == ORB_SLAM3::Tracking::OK) {
      const Sophus::SE3f Twc = Tcw.inverse();
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = StampFromNs(ns);
      odom.header.frame_id = "world";
      odom.child_frame_id = "cam0";
      odom.pose.pose = ToPoseMsg(Twc);
      pub_odom_->publish(odom);
    }

    // KF detection: publish a full snapshot whenever the map grows or switches.
    ORB_SLAM3::Atlas* atlas = slam_->GetAtlas();
    ORB_SLAM3::Map* pMap = atlas ? atlas->GetCurrentMap() : nullptr;
    if (!pMap) return;
    const uint64_t map_id = pMap->GetId();
    const uint64_t max_kf = pMap->GetMaxKFid();
    if (map_id != last_map_id_ || max_kf != last_max_kf_) {
      PublishNewKFImages(pMap, (map_id == last_map_id_) ? last_max_kf_ : 0, max_kf);
      PublishTrajectorySnapshot();
      PublishPoseGraph();
      last_map_id_ = map_id;
      last_max_kf_ = max_kf;
    }
  }

  void PublishNewKFImages(ORB_SLAM3::Map* pMap, uint64_t prev_max, uint64_t cur_max) {
    std::vector<ORB_SLAM3::KeyFrame*> kfs = pMap->GetAllKeyFrames();
    std::sort(kfs.begin(), kfs.end(), ORB_SLAM3::KeyFrame::lId);
    for (ORB_SLAM3::KeyFrame* kf : kfs) {
      if (!kf || kf->isBad()) continue;
      if (kf->mnId <= prev_max || kf->mnId > cur_max) continue;
      const int64_t kf_ns = StampNs(kf->mTimeStamp);
      auto it = ring_.find(kf_ns);
      if (it == ring_.end()) continue;  // image already evicted
      pub_image_->publish(*it->second);
    }
  }

  void PublishTrajectorySnapshot() {
    ORB_SLAM3::Atlas* atlas = slam_->GetAtlas();
    ORB_SLAM3::Map* pMap = atlas ? atlas->GetCurrentMap() : nullptr;
    if (!pMap) return;
    std::vector<ORB_SLAM3::KeyFrame*> kfs = pMap->GetAllKeyFrames();
    kfs.erase(std::remove_if(kfs.begin(), kfs.end(),
                             [](ORB_SLAM3::KeyFrame* kf) { return !kf || kf->isBad(); }),
              kfs.end());
    std::sort(kfs.begin(), kfs.end(), ORB_SLAM3::KeyFrame::lId);

    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = now();
    for (ORB_SLAM3::KeyFrame* kf : kfs) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.header.stamp = StampFromNs(StampNs(kf->mTimeStamp));
      ps.pose = ToPoseMsg(kf->GetPoseInverse());  // camera T_WC
      path.poses.push_back(ps);
    }
    pub_traj_->publish(path);
  }

  void PublishPoseGraph() {
    bool multi = false;
    auto msg = pgb_->Build(
        [this](double t) { return StampNs(t); }, now(), &multi);
    if (multi)
      RCLCPP_WARN(get_logger(), "atlas has >1 map; publishing active map only");
    if (!msg.vertex_id.empty())
      pub_graph_->publish(msg);
  }

  // Dense per-frame trajectory recovered from the tracker's relative poses,
  // port of System::SaveTrajectoryEuRoC (camera frame T_WC, no KF0 re-anchor so
  // it shares the world frame with /openmavis/trajectory).
  void PublishTrajectoryFinal() {
    ORB_SLAM3::Atlas* atlas = slam_->GetAtlas();
    if (!atlas) return;
    std::vector<ORB_SLAM3::Map*> maps = atlas->GetAllMaps();
    ORB_SLAM3::Map* big = nullptr;
    size_t best = 0;
    for (ORB_SLAM3::Map* m : maps) {
      const size_t n = m->GetAllKeyFrames().size();
      if (n > best) { best = n; big = m; }
    }
    if (!big) return;

    ORB_SLAM3::Tracking* tr = slam_->GetTracker();
    nav_msgs::msg::Path path;
    path.header.frame_id = "world";
    path.header.stamp = now();

    auto lit = tr->mlRelativeFramePoses.begin();
    auto lend = tr->mlRelativeFramePoses.end();
    auto lRit = tr->mlpReferences.begin();
    auto lT = tr->mlFrameTimes.begin();
    auto lbL = tr->mlbLost.begin();
    for (; lit != lend; ++lit, ++lRit, ++lT, ++lbL) {
      if (*lbL) continue;
      ORB_SLAM3::KeyFrame* pKF = *lRit;
      if (!pKF) continue;
      Sophus::SE3f Trw;
      while (pKF->isBad()) {
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
        if (!pKF) break;
      }
      if (!pKF || pKF->GetMap() != big) continue;
      Trw = Trw * pKF->GetPose();  // T_ref_w (no *Twb0 anchor)
      const Sophus::SE3f Twc = ((*lit) * Trw).inverse();  // camera T_WC

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.header.stamp = StampFromNs(NsFromSec(*lT));
      ps.pose = ToPoseMsg(Twc);
      path.poses.push_back(ps);
    }
    pub_traj_final_->publish(path);
    RCLCPP_INFO(get_logger(), "trajectory_final: %zu frames", path.poses.size());
  }

  // params
  std::string voc_file_, settings_file_, image_topic_, imu_topic_, output_ns_;
  double image_delay_ = 0.0;  // cam-IMU timeshift [s]

  // fisheye -> pinhole undistort (built lazily on the first frame)
  bool undistort_ = true;
  bool maps_ready_ = false;
  int und_w_ = 0, und_h_ = 0;
  cv::Mat fish_K_, fish_D_, new_K_, und_map1_, und_map2_;

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::unique_ptr<PoseGraphBuilder> pgb_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_final_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_;
  rclcpp::Publisher<okvis_pose_graph_msgs::msg::PoseGraph>::SharedPtr pub_graph_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;

  std::mutex imu_mtx_;
  std::condition_variable imu_cv_;
  std::deque<ORB_SLAM3::IMU::Point> imu_buf_;
  std::mutex img_mtx_;
  std::condition_variable img_cv_;
  std::deque<sensor_msgs::msg::CompressedImage::SharedPtr> img_queue_;
  std::thread worker_;
  std::atomic<bool> running_{false};

  std::mutex stamp_mtx_;
  std::unordered_map<uint64_t, int64_t> stamp_map_;  // key: double bits of t_sec

  // KF-image ring buffer (worker-thread only).
  static constexpr size_t kRingCap = 400;
  std::unordered_map<int64_t, sensor_msgs::msg::CompressedImage::SharedPtr> ring_;
  std::deque<int64_t> ring_order_;

  uint64_t last_map_id_ = UINT64_MAX;
  uint64_t last_max_kf_ = UINT64_MAX;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, OnSigint);
  std::signal(SIGTERM, OnSigint);

  auto node = std::make_shared<OrbSlam3Node>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_stop.load()) {
    exec.spin_some(std::chrono::milliseconds(5));
  }
  node->Finalize();
  rclcpp::shutdown();
  return 0;
}
