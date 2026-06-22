#pragma once
#include <array>

namespace drone_control {

double clamp_value(double value, double lower, double upper);
std::array<double, 3> quaternion_to_euler(double x, double y, double z, double w);

// rotation / matrix helpers
std::array<std::array<double,3>,3> R_from_quat(const std::array<double,4> & q);
std::array<std::array<double,3>,3> Rx(double phi);
std::array<std::array<double,3>,3> Ry(double theta);
std::array<std::array<double,3>,3> Rz(double psi);
std::array<std::array<double,3>,3> mat_mul(const std::array<std::array<double,3>,3> & A,
                                           const std::array<std::array<double,3>,3> & B);
std::array<std::array<double,3>,3> mat_transpose(const std::array<std::array<double,3>,3> & A);
std::array<double,3> vee(const std::array<std::array<double,3>,3> & S);

} // namespace drone_control
