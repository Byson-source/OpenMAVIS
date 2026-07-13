#pragma once
#include <vector>
#include <Eigen/Core>

namespace orb_slam3_ros2 {

struct EdgeInfo {
  double info_trans = 1.0;  // diagonal translation information (1/m^2)
  double info_rot = 1.0;    // diagonal rotation information (1/rad^2)
  int n_used = 0;           // co-observed landmarks that passed the gates
};

// Pixel-reprojection Gauss-Newton Hessian of the landmarks co-observed by two
// camera keyframes, w.r.t. a left perturbation [dt, dtheta] of the relative
// pose. Faithful port of OV-SLAM's KeyFrame::ComputeEdgeHessianInfo
// (ov_secondary_loop_fusion/src/keyframe.cpp:383-443): pinhole projection,
// depth gate 0.1-80 m, in-image gate, bidirectional average, geomean of the
// translation / rotation diagonal triples. Poses are camera-to-world (T_wc).
EdgeInfo ComputeEdgeHessianInfo(
    const std::vector<Eigen::Vector3d>& pts_w,
    const Eigen::Matrix3d& R_wc_a, const Eigen::Vector3d& t_wc_a,
    const Eigen::Matrix3d& R_wc_b, const Eigen::Vector3d& t_wc_b,
    double focal, double cols, double rows, double sigma_px);

}  // namespace orb_slam3_ros2
