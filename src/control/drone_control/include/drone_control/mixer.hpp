#pragma once
#include <vector>

namespace drone_control {

std::vector<double> quad_mix(double base, double roll_delta, double pitch_delta, double yaw_term);

} // namespace drone_control
