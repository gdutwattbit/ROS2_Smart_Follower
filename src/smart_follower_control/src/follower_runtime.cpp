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
  last_pose_message_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_pose_lock_id_ = -1;
  last_pose_lock_state_ = 0;
  last_pose_locked_track_found_ = false;
  last_pose_locked_track_confirmed_ = false;
  last_pose_locked_track_finite_ = false;
  last_invalid_reason_.clear();
  invalid_target_event_count_ = 0;
  reset_output();
}

void FollowerRuntime::reset_output()
{
  pid_r_.reset();
  pid_t_.reset();
  last_cmd_ = geometry_msgs::msg::Twist();
  last_raw_theta_ = 0.0;
  stale_target_hold_active_ = false;
  last_hold_zone_active_ = false;
}

namespace
{
void note_invalid_target(std::string & reason_slot, std::size_t & counter, const char * reason)
{
  reason_slot = reason;
  ++counter;
}
}  // namespace

void FollowerRuntime::on_pose(const smart_follower_msgs::msg::PersonPoseArray & msg)
{
  last_pose_message_stamp_ = rclcpp::Time(msg.header.stamp);
  last_pose_lock_id_ = msg.lock_id;
  last_pose_lock_state_ = static_cast<int>(msg.lock_state);
  last_pose_locked_track_found_ = false;
  last_pose_locked_track_confirmed_ = false;
  last_pose_locked_track_finite_ = false;

  if (msg.lock_id < 0) {
    stale_target_hold_active_ = false;
    last_target_.valid = false;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "lock_lost");
    return;
  }

  if (msg.lock_state != smart_follower_msgs::msg::PersonPoseArray::LOCKED) {
    stale_target_hold_active_ = false;
    last_target_.valid = false;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "lock_not_locked");
    return;
  }

  bool updated = false;
  bool matched_locked_track = false;
  bool locked_track_confirmed = false;
  bool locked_track_finite = false;
  for (const auto & person : msg.persons) {
    if (person.track_id != msg.lock_id) {
      continue;
    }

    matched_locked_track = true;
    last_pose_locked_track_found_ = true;
    if (person.track_state != smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
      break;
    }

    locked_track_confirmed = true;
    last_pose_locked_track_confirmed_ = true;
    if (!std::isfinite(person.position.x) || !std::isfinite(person.position.y)) {
      break;
    }

    locked_track_finite = true;
    last_pose_locked_track_finite_ = true;
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

    stale_target_hold_active_ = false;
    last_target_ = next_target;
    last_invalid_reason_.clear();
    updated = true;
    break;
  }

  if (!updated) {
    if (!matched_locked_track) {
      note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "locked_track_missing");
    } else if (!locked_track_confirmed) {
      note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "locked_track_not_confirmed");
    } else if (!locked_track_finite) {
      note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "locked_track_position_nan");
    } else {
      note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "locked_track_not_updated");
    }

    if (last_target_.valid) {
      stale_target_hold_active_ = true;
      last_target_.vx = 0.0;
      last_target_.vy = 0.0;
    } else {
      stale_target_hold_active_ = false;
      last_target_.vx = 0.0;
      last_target_.vy = 0.0;
    }
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
  const double raw_theta = std::atan2(target.y, target.x);
  last_raw_theta_ = raw_theta;

  double theta = raw_theta;
  if (std::abs(theta) < config_.theta_deadzone) {
    theta = 0.0;
  }

  const double dt = 1.0 / std::max(1.0, config_.control_rate);
  const double e_r = rho - config_.target_distance;
  const double e_t = theta;

  if (should_hold_stop(e_r, raw_theta, target)) {
    pid_r_.reset();
    pid_t_.reset();
    last_cmd_ = geometry_msgs::msg::Twist();
    last_hold_zone_active_ = true;
    return last_cmd_;
  }
  last_hold_zone_active_ = false;

  double v = pid_r_.update(e_r, dt, -config_.v_max, config_.v_max);
  double w = pid_t_.update(e_t, dt, -config_.w_max, config_.w_max);

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
  out.stale_target_hold = stale_target_hold_active_;
  out.hold_zone_active = last_hold_zone_active_;
  out.raw_theta = last_raw_theta_;
  out.last_pose_lock_id = last_pose_lock_id_;
  out.last_pose_lock_state = last_pose_lock_state_;
  out.locked_track_found = last_pose_locked_track_found_;
  out.locked_track_confirmed = last_pose_locked_track_confirmed_;
  out.locked_track_finite = last_pose_locked_track_finite_;
  out.invalid_event_count = invalid_target_event_count_;
  out.target_invalid_reason = last_invalid_reason_;

  if (!out.target_seen) {
    return out;
  }

  out.target_age_s = last_pose_message_stamp_.nanoseconds() > 0 ?
    std::max(0.0, (now_time - last_pose_message_stamp_).seconds()) : -1.0;
  out.target_valid = last_target_.valid && out.target_age_s >= 0.0;
  out.prediction_age_s = stale_target_hold_active_ ? 0.0 : std::min(
    std::max(0.0, (now_time - last_target_.stamp).seconds()),
    std::max(0.0, config_.prediction_horizon_s));
  out.predicted_target_valid = out.target_valid;
  if (out.target_valid && !stale_target_hold_active_) {
    out.target_invalid_reason.clear();
  }
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

  if (last_pose_message_stamp_.nanoseconds() == 0) {
    last_target_.valid = false;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "target_timeout");
    return std::nullopt;
  }

  const double message_age = (now_time - last_pose_message_stamp_).seconds();
  if (message_age < 0.0) {
    stale_target_hold_active_ = false;
    last_target_.valid = false;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "target_timeout");
    return std::nullopt;
  }

  if (stale_target_hold_active_) {
    return last_target_;
  }

  TargetState predicted = last_target_;
  const double age_used = std::min(
    std::max(0.0, (now_time - last_target_.stamp).seconds()),
    std::max(0.0, config_.prediction_horizon_s));
  predicted.x += predicted.vx * age_used;
  predicted.y += predicted.vy * age_used;
  return predicted;
}

double FollowerRuntime::rate_limit(double target, double current, double accel_limit, double dt) const
{
  const double delta_max = std::max(0.0, accel_limit) * std::max(1e-3, dt);
  return std::clamp(target, current - delta_max, current + delta_max);
}

bool FollowerRuntime::should_hold_stop(double distance_error, double raw_theta, const TargetState & target) const
{
  if (config_.stop_hold_distance <= 0.0 || config_.stop_hold_angle <= 0.0) {
    return false;
  }

  return std::abs(distance_error) <= config_.stop_hold_distance &&
         std::abs(raw_theta) <= config_.stop_hold_angle &&
         std::hypot(target.vx, target.vy) <= std::max(0.0, config_.stop_hold_speed_mps);
}

}  // namespace smart_follower_control

