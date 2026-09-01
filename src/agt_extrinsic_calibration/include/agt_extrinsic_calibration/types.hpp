#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace agt_extrinsic_calibration {

struct PoseStamped {
  double timestamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

using Trajectory = std::vector<PoseStamped>;

struct PosePair {
  Eigen::Isometry3d wheel_delta{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d lio_delta{Eigen::Isometry3d::Identity()};
  double timestamp{0.0};
};

}  // namespace agt_extrinsic_calibration
