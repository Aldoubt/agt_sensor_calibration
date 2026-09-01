#include "agt_extrinsic_calibration/trajectory.hpp"
#include <gtest/gtest.h>

using namespace agt_extrinsic_calibration;

TEST(Trajectory, InterpolatesPositionAndRotation) {
  PoseStamped a; a.timestamp = 0.0; a.position = Eigen::Vector3d::Zero();
  PoseStamped b; b.timestamp = 1.0; b.position = Eigen::Vector3d(1, 2, 3);
  b.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  Trajectory t{a, b};
  const auto p = interpolate(t, 0.5);
  EXPECT_NEAR(p.position.x(), 0.5, 1e-12);
  EXPECT_NEAR(p.position.y(), 1.0, 1e-12);
  EXPECT_NEAR(p.position.z(), 1.5, 1e-12);
  EXPECT_NEAR(std::abs(p.orientation.w()), std::sqrt(0.5), 1e-12);
}

TEST(Trajectory, RejectsOutsideRange) {
  Trajectory t{{0.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
               {1.0, Eigen::Vector3d::Ones(), Eigen::Quaterniond::Identity()}};
  EXPECT_THROW(interpolate(t, -0.1), std::out_of_range);
}
