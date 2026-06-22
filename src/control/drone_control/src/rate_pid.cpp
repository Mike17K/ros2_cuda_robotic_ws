#include "drone_control/rate_pid.hpp"
#include <algorithm>
#include <cmath>

namespace drone_control {

RatePID::RatePID()
: kp_(0.0), ki_(0.0), kd_(0.0), integral_(0.0), prev_error_(0.0), integral_limit_(0.0), integral_leak_rate_(0.0)
{}

void RatePID::set_gains(double kp, double ki, double kd){
  kp_ = kp; ki_ = ki; kd_ = kd;
}

void RatePID::set_integral_limits(double limit, double leak_rate){
  integral_limit_ = std::max(0.0, limit);
  integral_leak_rate_ = std::max(0.0, leak_rate);
}

void RatePID::set_integral(double integral){ integral_ = integral; }

void RatePID::reset(){ integral_ = 0.0; prev_error_ = 0.0; }

double RatePID::update(double error, double dt){
  if (dt <= 0.0) return 0.0;

  if (integral_leak_rate_ > 0.0) {
    integral_ *= std::exp(-integral_leak_rate_ * dt);
  }

  integral_ += error * dt;
  if (integral_limit_ > 0.0) {
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
  }

  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;
  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

} // namespace drone_control
