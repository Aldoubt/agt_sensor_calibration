#pragma once

#include "agt_extrinsic_calibration/types.hpp"
#include <string>

namespace agt_extrinsic_calibration {

class RosbagReader {
public:
  Trajectory read_odometry(const std::string &bag_uri, const std::string &topic) const;
  void write_csv(const Trajectory &trajectory, const std::string &filename) const;
};

}  // namespace agt_extrinsic_calibration
