#pragma once

#include <Eigen/Geometry>
#include <string>

namespace agt_extrinsic_calibration {

struct ExtrinsicPrior {
  std::string parent_frame{"base_link"};
  std::string child_frame{"mid360_link"};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double translation_sigma{0.05};
  double rotation_sigma_rad{0.0872664626};
};

bool load_extrinsic_prior(const std::string &yaml_file,
                          ExtrinsicPrior &prior);

}
