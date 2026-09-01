#include "agt_extrinsic_calibration/prior_loader.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace agt_extrinsic_calibration {

ExtrinsicPrior load_extrinsic_prior(const std::string &yaml_file)
{
  ExtrinsicPrior prior;

  auto node = YAML::LoadFile(yaml_file);

  prior.enabled = true;

  auto t = node["translation"];
  if (t) {
    prior.transform.translation().x() = t["x"].as<double>(0.0);
    prior.transform.translation().y() = t["y"].as<double>(0.0);
    prior.transform.translation().z() = t["z"].as<double>(0.0);
  }

  auto r = node["rotation"];
  if (r) {
    Eigen::Quaterniond q(
      r["qw"].as<double>(1.0),
      r["qx"].as<double>(0.0),
      r["qy"].as<double>(0.0),
      r["qz"].as<double>(0.0));
    prior.transform.linear() = q.normalized().toRotationMatrix();
  }

  auto sigma = node["sigma"];
  if (sigma) {
    prior.translation_sigma = sigma["translation_m"].as<double>(0.05);
    const double deg = sigma["rotation_deg"].as<double>(5.0);
    prior.rotation_sigma_rad = deg * M_PI / 180.0;
  }

  return prior;
}

}
