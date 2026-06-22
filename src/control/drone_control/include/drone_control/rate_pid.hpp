#pragma once

namespace drone_control {

class RatePID {
public:
  RatePID();
  void set_gains(double kp, double ki, double kd);
  void set_integral_limits(double limit, double leak_rate);
  void set_integral(double integral);
  void reset();
  double update(double error, double dt);
private:
  double kp_, ki_, kd_;
  double integral_, prev_error_;
  double integral_limit_;
  double integral_leak_rate_;
};

} // namespace drone_control
