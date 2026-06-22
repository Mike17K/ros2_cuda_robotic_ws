#include <vector>

namespace drone_control {

std::vector<double> quad_mix(double base, double roll_delta, double pitch_delta, double yaw_term){
  std::vector<double> m(4, base);
  m[0] +=  -roll_delta + pitch_delta - yaw_term; // front_left
  m[1] += roll_delta - pitch_delta - yaw_term; // rear_right (cw)
  m[2] += roll_delta + pitch_delta + yaw_term; // front_right (ccw)
  m[3] +=  -roll_delta - pitch_delta + yaw_term; // rear_left (ccw)
  return m;
}

} // namespace drone_control
