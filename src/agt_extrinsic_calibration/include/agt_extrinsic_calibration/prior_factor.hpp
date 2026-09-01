#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace agt_extrinsic_calibration {

/**
 * Prior residual for extrinsic calibration.
 *
 * The optimization state is:
 *
 *   T_base_lidar
 *
 * and the prior constrains the estimate around a measured/CAD/tf_static
 * initial transform.
 */
struct ExtrinsicPriorFactorConfig {
  Eigen::Isometry3d T_prior{Eigen::Isometry3d::Identity()};
  double translation_weight{1.0};
  double rotation_weight{1.0};
};

}  // namespace agt_extrinsic_calibration
