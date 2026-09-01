#pragma once

#include "agt_extrinsic_calibration/optimizer.hpp"
#include <string>

namespace agt_extrinsic_calibration {

ExtrinsicPrior load_extrinsic_prior(const std::string &yaml_file);

}
