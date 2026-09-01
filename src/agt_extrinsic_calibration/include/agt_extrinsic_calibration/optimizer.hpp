#pragma once

#include "agt_extrinsic_calibration/types.hpp"
#include <string>

namespace agt_extrinsic_calibration {

struct ExtrinsicPrior {
  bool enabled{false};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double translation_sigma{0.05};
  double rotation_sigma_rad{0.0872664626};
};

struct OptimizationReport {
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double initial_rmse{0.0};
  double final_rmse{0.0};
  int iterations{0};
  int pairs{0};
  bool success{false};
  std::string message;
};

OptimizationReport optimize_extrinsic(const std::vector<PosePair> &pairs);
OptimizationReport optimize_extrinsic(const std::vector<PosePair> &pairs,
                                      const ExtrinsicPrior &prior);
void write_result(const OptimizationReport &report, const std::string &yaml_file,
                  const std::string &markdown_file);

}  // namespace agt_extrinsic_calibration
