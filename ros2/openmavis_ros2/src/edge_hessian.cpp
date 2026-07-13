#include "edge_hessian.hpp"

#include <cmath>
#include <algorithm>

namespace orb_slam3_ros2 {

namespace {
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0.0, -v.z(),  v.y(),
        v.z(),     0.0, -v.x(),
       -v.y(),  v.x(),     0.0;
  return m;
}
}  // namespace

EdgeInfo ComputeEdgeHessianInfo(
    const std::vector<Eigen::Vector3d>& pts_w,
    const Eigen::Matrix3d& R_wc_a, const Eigen::Vector3d& t_wc_a,
    const Eigen::Matrix3d& R_wc_b, const Eigen::Vector3d& t_wc_b,
    double focal, double cols, double rows, double sigma_px) {
  EdgeInfo out;
  if (pts_w.empty() || focal <= 0.0 || sigma_px <= 0.0)
    return out;

  const double w = 1.0 / (sigma_px * sigma_px);
  constexpr double kMinDepth = 0.1;   // behind / degenerate
  constexpr double kMaxDepth = 80.0;  // far points deflate the geomean

  auto accumulate = [&](const Eigen::Matrix3d& R_wc, const Eigen::Vector3d& t_wc,
                        Eigen::Matrix<double, 6, 6>& JtJ) -> int {
    int used = 0;
    for (const auto& p_w : pts_w) {
      const Eigen::Vector3d p = R_wc.transpose() * (p_w - t_wc);
      if (p.z() < kMinDepth || p.z() > kMaxDepth)
        continue;
      const double iz = 1.0 / p.z();
      if (cols > 0.0 && rows > 0.0) {
        const double u = focal * p.x() * iz + cols / 2.0;
        const double v = focal * p.y() * iz + rows / 2.0;
        if (u < 0.0 || u > cols - 1.0 || v < 0.0 || v > rows - 1.0)
          continue;
      }
      Eigen::Matrix<double, 2, 3> Jp;
      Jp << focal * iz, 0.0, -focal * p.x() * iz * iz,
            0.0, focal * iz, -focal * p.y() * iz * iz;
      Eigen::Matrix<double, 2, 6> J;
      J.leftCols<3>() = Jp;
      J.rightCols<3>() = -Jp * skew(p);
      JtJ.noalias() += w * J.transpose() * J;
      ++used;
    }
    return used;
  };

  Eigen::Matrix<double, 6, 6> JtJ_a = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> JtJ_b = Eigen::Matrix<double, 6, 6>::Zero();
  const int used_a = accumulate(R_wc_a, t_wc_a, JtJ_a);
  const int used_b = accumulate(R_wc_b, t_wc_b, JtJ_b);
  out.n_used = std::min(used_a, used_b);
  if (out.n_used <= 0)
    return out;

  const Eigen::Matrix<double, 6, 1> diag =
      (0.5 * (JtJ_a + JtJ_b)).diagonal().cwiseMax(1e-6);
  out.info_trans = std::exp(diag.head<3>().array().log().mean());
  out.info_rot = std::exp(diag.tail<3>().array().log().mean());
  return out;
}

}  // namespace orb_slam3_ros2
