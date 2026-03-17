#include "smart_follower_control/follower_runtime.hpp"

#include <algorithm>
#include <cmath>

namespace smart_follower_control
{

void FollowerRuntime::set_config(const FollowerRuntimeConfig & config)
{
  config_ = config;
  configure_controllers();
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

    last_target_.x = person.position.x;
    last_target_.y = person.position.y;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
    last_target_.stamp = rclcpp::Time(msg.header.stamp);
    last_target_.valid = true;
    updated = true;
    break;
  }

  if (!updated) {
    last_target_.valid = false;
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

FollowerRuntimeSnapshot FollowerRuntime::snapshot() const
{
  FollowerRuntimeSnapshot out;
  out.last_cmd_v = last_cmd_.linear.x;
  out.last_cmd_w = last_cmd_.angular.z;
  out.target_valid = last_target_.valid;
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

  return last_target_;
}

double FollowerRuntime::rate_limit(double target, double current, double accel_limit, double dt) const
{
  const double delta_max = std::max(0.0, accel_limit) * std::max(1e-3, dt);
  return std::clamp(target, current - delta_max, current + delta_max);
}

}  // namespace smart_follower_control
