#pragma once

#include <algorithm>

namespace smart_follower_control
{

class PID
{
public:
  void configure(double kp, double ki, double kd, double i_limit, double kaw)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_limit_ = std::max(0.0, i_limit);
    kaw_ = kaw;
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    initialized_ = false;
  }

  double update(double error, double dt, double u_min, double u_max)
  {
    if (dt <= 1e-6) {
      dt = 1e-3;
    }

    if (!initialized_) {
      prev_error_ = error;
      initialized_ = true;
    }

    const double derivative = (error - prev_error_) / dt;
    double u_raw = kp_ * error + ki_ * integral_ + kd_ * derivative;
    const double u_sat = std::clamp(u_raw, u_min, u_max);

    const bool would_worsen = (u_raw > u_max && error > 0.0) || (u_raw < u_min && error < 0.0);
    if (!would_worsen) {
      integral_ += error * dt;
      integral_ = std::clamp(integral_, -i_limit_, i_limit_);
    } else {
      integral_ += kaw_ * (u_sat - u_raw) * dt;
      integral_ = std::clamp(integral_, -i_limit_, i_limit_);
    }

    prev_error_ = error;
    return u_sat;
  }

private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double i_limit_{0.5};
  double kaw_{0.1};

  double integral_{0.0};
  double prev_error_{0.0};
  bool initialized_{false};
};

}  // namespace smart_follower_control
