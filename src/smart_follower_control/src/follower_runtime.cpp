#include "smart_follower_control/follower_runtime.hpp"

#include <algorithm>
#include <cmath>

namespace smart_follower_control
{

void FollowerRuntime::set_config(const FollowerRuntimeConfig & config)
{
  config_ = config;
  configure_controllers();
  clamp_target_speed(last_target_, config_.max_target_speed_mps);
}

void FollowerRuntime::clear()
{
  last_target_ = TargetState();
  reset_output();
}

void FollowerRuntime::reset_output()
{
  pid_r_.reset();
  pid_t_.reset();
  last_cmd_ = geometry_msgs::msg::Twist();
}

void FollowerRuntime::on_pose(const smart_follower_msgs::msg::PersonPoseArray & msg)
{
  if (msg.lock_id < 0) {
    last_target_.valid = false;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
    return;
  }

  bool updated = false;
  for (const auto & person : msg.persons) {
    if (person.track_id != msg.lock_id) {
      continue;
    }
    if (person.track_state != smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
      continue;
    }
    if (!std::isfinite(person.position.x) || !std::isfinite(person.position.y)) {
      continue;
    }

    TargetState next_target;
    next_target.x = person.position.x;
    next_target.y = person.position.y;
    next_target.stamp = rclcpp::Time(msg.header.stamp);
    next_target.valid = true;

    if (last_target_.valid) {
      const double dt = (next_target.stamp - last_target_.stamp).seconds();
      if (dt > 1e-3 && dt <= config_.target_timeout) {
        const double vx_meas = (next_target.x - last_target_.x) / dt;
        const double vy_meas = (next_target.y - last_target_.y) / dt;
        const double alpha = std::clamp(config_.velocity_ema_alpha, 0.0, 1.0);
        next_target.vx = alpha * last_target_.vx + (1.0 - alpha) * vx_meas;
        next_target.vy = alpha * last_target_.vy + (1.0 - alpha) * vy_meas;
      }
    }

    clamp_target_speed(next_target, config_.max_target_speed_mps);
    last_target_ = next_target;
    updated = true;
    break;
  }

  if (!updated) {
    last_target_.valid = false;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
  }
}

geometry_msgs::msg::Twist FollowerRuntime::compute_command(const rclcpp::Time & now_time)
{
  auto target_opt = predict_target(now_time);
  if (!target_opt.has_value()) {
    reset_output();
    return last_cmd_;
  }

  const auto & target = target_opt.value();
  const double rho = std::sqrt(target.x * target.x + target.y * target.y);
  double theta = std::atan2(target.y, target.x);
  if (std::abs(theta) < config_.theta_deadzone) {
    theta = 0.0;
  }

  const double dt = 1.0 / std::max(1.0, config_.control_rate);
  const double e_r = rho - config_.target_distance;
  const double e_t = theta;

  double v = pid_r_.update(e_r, dt, -config_.v_max, config_.v_max);
  double w = pid_t_.update(e_t, dt, -config_.w_max, config_.w_max);

  if (std::abs(theta) > 0.5) {
    v *= 0.3;
  }

  v = rate_limit(v, last_cmd_.linear.x, config_.dv_max, dt);
  w = rate_limit(w, last_cmd_.angular.z, config_.dw_max, dt);

  last_cmd_.linear.x = v;
  last_cmd_.angular.z = w;
  return last_cmd_;
}

FollowerRuntimeSnapshot FollowerRuntime::snapshot(const rclcpp::Time & now_time) const
{
  FollowerRuntimeSnapshot out;
  out.last_cmd_v = last_cmd_.linear.x;
  out.last_cmd_w = last_cmd_.angular.z;
  out.target_seen = last_target_.stamp.nanoseconds() > 0;
  out.target_vx = last_target_.vx;
  out.target_vy = last_target_.vy;
  out.target_speed_mps = std::hypot(last_target_.vx, last_target_.vy);

  if (!out.target_seen) {
    return out;
  }

  out.target_age_s = std::max(0.0, (now_time - last_target_.stamp).seconds());
  out.target_valid = last_target_.valid && out.target_age_s <= config_.target_timeout;
  out.prediction_age_s = std::min(out.target_age_s, std::max(0.0, config_.prediction_horizon_s));
  out.predicted_target_valid = out.target_valid;
  return out;
}

void FollowerRuntime::configure_controllers()
{
  pid_r_.configure(config_.kp_r, config_.ki_r, config_.kd_r, config_.i_limit, config_.kaw);
  pid_t_.configure(config_.kp_t, config_.ki_t, config_.kd_t, config_.i_limit, config_.kaw);
}

std::optional<FollowerRuntime::TargetState> FollowerRuntime::predict_target(const rclcpp::Time & now_time)
{
  if (!last_target_.valid) {
    return std::nullopt;
  }

  const double age = (now_time - last_target_.stamp).seconds();
  if (age < 0.0 || age > config_.target_timeout) {
    last_target_.valid = false;
    return std::nullopt;
  }

  TargetState predicted = last_target_;
  const double age_used = std::min(age, std::max(0.0, config_.prediction_horizon_s));
  predicted.x += predicted.vx * age_used;
  predicted.y += predicted.vy * age_used;
  return predicted;
}

double FollowerRuntime::rate_limit(double target, double current, double accel_limit, double dt) const
{
  const double delta_max = std::max(0.0, accel_limit) * std::max(1e-3, dt);
  return std::clamp(target, current - delta_max, current + delta_max);
}

void FollowerRuntime::clamp_target_speed(TargetState & target, double max_speed_mps)
{
  const double speed = std::hypot(target.vx, target.vy);
  if (max_speed_mps <= 0.0 || speed <= max_speed_mps || speed <= 1e-9) {
    return;
  }

  const double scale = max_speed_mps / speed;
  target.vx *= scale;
  target.vy *= scale;
}

}  // namespace smart_follower_control
