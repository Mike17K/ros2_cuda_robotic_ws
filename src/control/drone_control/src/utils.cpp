#include "drone_control/utils.hpp"
#include <cmath>

namespace drone_control {

double clamp_value(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

std::array<double, 3> quaternion_to_euler(double x, double y, double z, double w)
{
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  double pitch = 0.0;
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

std::array<std::array<double,3>,3> R_from_quat(const std::array<double,4> & q) {
  double x=q[0], y=q[1], z=q[2], w=q[3];
  std::array<std::array<double,3>,3> R{};
  R[0][0] = 1 - 2*(y*y + z*z);
  R[0][1] = 2*(x*y - z*w);
  R[0][2] = 2*(x*z + y*w);
  R[1][0] = 2*(x*y + z*w);
  R[1][1] = 1 - 2*(x*x + z*z);
  R[1][2] = 2*(y*z - x*w);
  R[2][0] = 2*(x*z - y*w);
  R[2][1] = 2*(y*z + x*w);
  R[2][2] = 1 - 2*(x*x + y*y);
  return R;
}

std::array<std::array<double,3>,3> Rx(double phi) {
  std::array<std::array<double,3>,3> R{};
  double c = std::cos(phi), s = std::sin(phi);
  R[0] = {1.0, 0.0, 0.0};
  R[1] = {0.0, c, -s};
  R[2] = {0.0, s, c};
  return R;
}

std::array<std::array<double,3>,3> Ry(double theta) {
  std::array<std::array<double,3>,3> R{};
  double c = std::cos(theta), s = std::sin(theta);
  R[0] = {c, 0.0, s};
  R[1] = {0.0, 1.0, 0.0};
  R[2] = {-s, 0.0, c};
  return R;
}

std::array<std::array<double,3>,3> Rz(double psi) {
  std::array<std::array<double,3>,3> R{};
  double c = std::cos(psi), s = std::sin(psi);
  R[0] = {c, -s, 0.0};
  R[1] = {s, c, 0.0};
  R[2] = {0.0, 0.0, 1.0};
  return R;
}

std::array<std::array<double,3>,3> mat_mul(const std::array<std::array<double,3>,3> & A,
                                           const std::array<std::array<double,3>,3> & B) {
  std::array<std::array<double,3>,3> C{};
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
    C[i][j]=0.0;
    for (int k=0;k<3;++k) C[i][j]+=A[i][k]*B[k][j];
  }
  return C;
}

std::array<std::array<double,3>,3> mat_transpose(const std::array<std::array<double,3>,3> & A){
  std::array<std::array<double,3>,3> T{};
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) T[i][j]=A[j][i];
  return T;
}

std::array<double,3> vee(const std::array<std::array<double,3>,3> & S){
  return std::array<double,3>{S[2][1], S[0][2], S[1][0]};
}

} // namespace drone_control
