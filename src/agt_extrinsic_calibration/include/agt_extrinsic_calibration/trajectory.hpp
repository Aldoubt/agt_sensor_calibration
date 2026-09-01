#pragma once

#include "agt_extrinsic_calibration/types.hpp"

namespace agt_extrinsic_calibration {

PoseStamped interpolate(const Trajectory &trajectory, double timestamp);
std::vector<PosePair> make_motion_pairs(const Trajectory &wheel, const Trajectory &lio,
                                        double sample_period, double min_motion);

}  // namespace agt_extrinsic_calibration
