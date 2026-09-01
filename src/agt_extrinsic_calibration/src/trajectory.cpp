#include "agt_extrinsic_calibration/trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agt_extrinsic_calibration {
PoseStamped interpolate(const Trajectory &tr, double t) {
  if (tr.empty() || t < tr.front().timestamp || t > tr.back().timestamp) throw std::out_of_range("time outside trajectory");
  auto it = std::lower_bound(tr.begin(), tr.end(), t, [](const PoseStamped &p, double v) { return p.timestamp < v; });
  if (it == tr.begin() || it->timestamp == t) return *it;
  const auto &b = *it; const auto &a = *(it - 1); const double u = (t - a.timestamp) / (b.timestamp - a.timestamp);
  PoseStamped p; p.timestamp = t; p.position = a.position + u * (b.position - a.position);
  p.orientation = a.orientation.slerp(u, b.orientation).normalized(); return p;
}

std::vector<PosePair> make_motion_pairs(const Trajectory &wheel, const Trajectory &lio,
                                        double sample_period, double min_motion) {
  if (wheel.size() < 2 || lio.size() < 2) return {};
  const double start = std::max(wheel.front().timestamp, lio.front().timestamp);
  const double end = std::min(wheel.back().timestamp, lio.back().timestamp);
  std::vector<PosePair> result;
  double previous = start;
  for (double t = start + sample_period; t <= end; t += sample_period) {
    const auto w0 = interpolate(wheel, previous), w1 = interpolate(wheel, t);
    const auto l0 = interpolate(lio, previous), l1 = interpolate(lio, t);
    Eigen::Isometry3d W0 = Eigen::Translation3d(w0.position) * w0.orientation;
    Eigen::Isometry3d W1 = Eigen::Translation3d(w1.position) * w1.orientation;
    Eigen::Isometry3d L0 = Eigen::Translation3d(l0.position) * l0.orientation;
    Eigen::Isometry3d L1 = Eigen::Translation3d(l1.position) * l1.orientation;
    Eigen::Isometry3d wd = W0.inverse() * W1, ld = L0.inverse() * L1;
    if (wd.translation().norm() + Eigen::AngleAxisd(wd.rotation()).angle() > min_motion) {
      result.push_back({wd, ld, t});
    }
    previous = t;
  }
  return result;
}
}  // namespace agt_extrinsic_calibration
