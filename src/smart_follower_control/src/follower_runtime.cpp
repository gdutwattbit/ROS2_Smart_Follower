#include "smart_follower_control/follower_runtime.hpp"

#include <algorithm>
#include <cmath>

namespace smart_follower_control
{

void FollowerRuntime::set_config(const FollowerRuntimeConfig & config)
{
  config_ = config;
  configure_controllers();
  reset_steering_filter();
}

void FollowerRuntime::clear()
{
  last_target_ = TargetState();
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
  last_filtered_theta_ = 0.0;
  reset_steering_filter();
}

namespace
{
void note_invalid_target(std::string & reason_slot, std::size_t & counter, const char * reason)
{
  reason_slot = reason;
  ++counter;
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

void FollowerRuntime::on_pose(const smart_follower_msgs::msg::PersonPoseArray & msg)
{
  const int previous_lock_id = last_pose_lock_id_;
  last_pose_lock_id_ = msg.lock_id;
  last_pose_lock_state_ = static_cast<int>(msg.lock_state);
  last_pose_locked_track_found_ = false;
  last_pose_locked_track_confirmed_ = false;
  last_pose_locked_track_finite_ = false;

  if (msg.lock_id != previous_lock_id) {
    reset_steering_filter();
  }

  if (msg.lock_id < 0) {
    last_target_.valid = false;
    last_target_.vx = 0.0;
    last_target_.vy = 0.0;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "lock_lost");
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

    if (!last_target_.valid) {
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
  last_raw_theta_ = std::atan2(target.y, target.x);

  const double theta = filter_theta_measurement(last_raw_theta_, now_time);
  last_filtered_theta_ = theta;

  const double dt = 1.0 / std::max(1.0, config_.control_rate);
  const double e_r = rho - config_.target_distance;
  const double e_t = theta;

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
  out.steering_filter_ready = steering_filter_.initialized;
  out.raw_theta = last_raw_theta_;
  out.filtered_theta = last_filtered_theta_;
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

  out.target_age_s = std::max(0.0, (now_time - last_target_.stamp).seconds());
  out.target_valid = last_target_.valid && out.target_age_s <= config_.target_timeout;
  out.prediction_age_s = std::min(out.target_age_s, std::max(0.0, config_.prediction_horizon_s));
  out.predicted_target_valid = out.target_valid;
  if (out.target_valid) {
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

  const double age = (now_time - last_target_.stamp).seconds();
  if (age < 0.0 || age > config_.target_timeout) {
    last_target_.valid = false;
    note_invalid_target(last_invalid_reason_, invalid_target_event_count_, "target_timeout");
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

void FollowerRuntime::reset_steering_filter()
{
  const double initial_covariance = std::max(1e-6, config_.steering_kalman_initial_covariance);
  steering_filter_ = SteeringKalmanState();
  steering_filter_.p00 = initial_covariance;
  steering_filter_.p11 = initial_covariance;
}

double FollowerRuntime::filter_theta_measurement(double theta, const rclcpp::Time & stamp)
{
  const double initial_covariance = std::max(1e-6, config_.steering_kalman_initial_covariance);
  if (!steering_filter_.initialized) {
    steering_filter_.initialized = true;
    steering_filter_.theta = theta;
    steering_filter_.theta_rate = 0.0;
    steering_filter_.p00 = initial_covariance;
    steering_filter_.p01 = 0.0;
    steering_filter_.p10 = 0.0;
    steering_filter_.p11 = initial_covariance;
    steering_filter_.stamp = stamp;
    return theta;
  }

  double dt = (stamp - steering_filter_.stamp).seconds();
  if (!std::isfinite(dt) || dt <= 1e-3) {
    dt = 1.0 / std::max(1.0, config_.control_rate);
  }
  dt = std::clamp(dt, 1e-3, 0.25);

  const double process_noise = std::max(1e-6, config_.steering_kalman_process_noise);
  const double measurement_noise = std::max(1e-6, config_.steering_kalman_measurement_noise);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  const double theta_pred = normalize_angle(steering_filter_.theta + dt * steering_filter_.theta_rate);
  const double theta_rate_pred = steering_filter_.theta_rate;

  const double q00 = 0.25 * dt4 * process_noise;
  const double q01 = 0.5 * dt3 * process_noise;
  const double q11 = dt2 * process_noise;

  const double p00_pred = steering_filter_.p00 + dt * (steering_filter_.p10 + steering_filter_.p01) + dt2 * steering_filter_.p11 + q00;
  const double p01_pred = steering_filter_.p01 + dt * steering_filter_.p11 + q01;
  const double p10_pred = steering_filter_.p10 + dt * steering_filter_.p11 + q01;
  const double p11_pred = steering_filter_.p11 + q11;

  const double innovation = normalize_angle(theta - theta_pred);
  const double innovation_covariance = p00_pred + measurement_noise;
  if (!std::isfinite(innovation_covariance) || innovation_covariance <= 1e-9) {
    steering_filter_.initialized = false;
    return filter_theta_measurement(theta, stamp);
  }

  const double k0 = p00_pred / innovation_covariance;
  const double k1 = p10_pred / innovation_covariance;

  steering_filter_.theta = normalize_angle(theta_pred + k0 * innovation);
  steering_filter_.theta_rate = theta_rate_pred + k1 * innovation;

  const double p00 = (1.0 - k0) * p00_pred;
  const double p01 = (1.0 - k0) * p01_pred;
  const double p10 = p10_pred - k1 * p00_pred;
  const double p11 = p11_pred - k1 * p01_pred;
  const double p01_sym = 0.5 * (p01 + p10);

  steering_filter_.p00 = std::max(1e-9, p00);
  steering_filter_.p01 = p01_sym;
  steering_filter_.p10 = p01_sym;
  steering_filter_.p11 = std::max(1e-9, p11);
  steering_filter_.stamp = stamp;
  return steering_filter_.theta;
}

}  // namespace smart_follower_control
