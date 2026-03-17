#include "smart_follower_control/arbiter_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace smart_follower_control
{

void ArbiterRuntime::set_config(const ArbiterRuntimeConfig & config)
{
  config_ = config;
}

void ArbiterRuntime::activate()
{
  mode_ = ArbiterMode::STOP;
}

void ArbiterRuntime::clear()
{
  last_target_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_avoid_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  avoid_zero_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_follow_cmd_ = geometry_msgs::msg::Twist();
  latest_avoid_cmd_ = geometry_msgs::msg::Twist();
  last_target_theta_ = 0.0;
  avoid_nonzero_count_ = 0;
  avoid_zero_count_ = 0;
  avoid_latched_ = false;
  stop_latched_ = false;
  mode_ = ArbiterMode::STOP;
}

void ArbiterRuntime::on_person_pose(const smart_follower_msgs::msg::PersonPoseArray & msg)
{
  if (msg.lock_id < 0 || msg.lock_state != smart_follower_msgs::msg::PersonPoseArray::LOCKED) {
    return;
  }

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
    last_target_time_ = rclcpp::Time(msg.header.stamp);
    last_target_theta_ = std::atan2(person.position.y, person.position.x);
    break;
  }
}

void ArbiterRuntime::on_follow_cmd(const geometry_msgs::msg::Twist & msg)
{
  latest_follow_cmd_ = msg;
}

void ArbiterRuntime::on_avoid_cmd(const geometry_msgs::msg::Twist & msg, const rclcpp::Time & now_time)
{
  latest_avoid_cmd_ = msg;
  last_avoid_time_ = now_time;

  if (is_nonzero(latest_avoid_cmd_)) {
    avoid_nonzero_count_++;
    avoid_zero_count_ = 0;
    if (avoid_nonzero_count_ >= config_.avoid_enter_threshold) {
      avoid_latched_ = true;
    }
    return;
  }

  avoid_zero_count_++;
  avoid_nonzero_count_ = 0;
  if (avoid_zero_count_ == 1) {
    avoid_zero_start_ = now_time;
  }
}

void ArbiterRuntime::on_user_cmd(const smart_follower_msgs::msg::FollowCommand & msg)
{
  if (msg.command == smart_follower_msgs::msg::FollowCommand::ESTOP) {
    stop_latched_ = true;
    mode_ = ArbiterMode::STOP;
    latest_follow_cmd_ = geometry_msgs::msg::Twist();
    latest_avoid_cmd_ = geometry_msgs::msg::Twist();
    return;
  }

  if (msg.command == smart_follower_msgs::msg::FollowCommand::RESET) {
    stop_latched_ = false;
    mode_ = ArbiterMode::STOP;
    avoid_nonzero_count_ = 0;
    avoid_zero_count_ = 0;
    avoid_latched_ = false;
    latest_follow_cmd_ = geometry_msgs::msg::Twist();
    latest_avoid_cmd_ = geometry_msgs::msg::Twist();
  }
}

geometry_msgs::msg::Twist ArbiterRuntime::compute_output(const rclcpp::Time & now_time)
{
  const bool avoid_on = avoid_valid(now_time);

  double age = std::numeric_limits<double>::infinity();
  if (last_target_time_.nanoseconds() > 0) {
    age = (now_time - last_target_time_).seconds();
  }

  if (!stop_latched_ && age > config_.thresholds.lost_time_search_max) {
    stop_latched_ = true;
  }

  if (stop_latched_) {
    mode_ = ArbiterMode::STOP;
  } else if (avoid_on) {
    mode_ = ArbiterMode::AVOID;
  } else {
    mode_ = select_follow_mode(age, config_.thresholds);
  }

  geometry_msgs::msg::Twist out;
  switch (mode_) {
    case ArbiterMode::FOLLOW_NORMAL:
      out = latest_follow_cmd_;
      break;
    case ArbiterMode::FOLLOW_DEGRADED:
      out = latest_follow_cmd_;
      out.linear.x *= config_.degraded_linear_scale;
      if (!std::isfinite(out.angular.z) || std::abs(out.angular.z) < 1e-6) {
        out.angular.z = std::clamp(last_target_theta_, -0.8, 0.8);
      }
      break;
    case ArbiterMode::SEARCH:
      out.linear.x = 0.0;
      out.angular.z = config_.search_angular_speed;
      break;
    case ArbiterMode::AVOID:
      out = latest_avoid_cmd_;
      break;
    case ArbiterMode::STOP:
    default:
      out = geometry_msgs::msg::Twist();
      break;
  }

  return out;
}

ArbiterRuntimeSnapshot ArbiterRuntime::snapshot(const rclcpp::Time & now_time) const
{
  ArbiterRuntimeSnapshot out;
  out.mode = mode_;
  out.stop_latched = stop_latched_;
  out.avoid_latched = avoid_latched_;
  if (last_target_time_.nanoseconds() > 0) {
    out.last_target_age_s = (now_time - last_target_time_).seconds();
  }
  return out;
}

bool ArbiterRuntime::is_nonzero(const geometry_msgs::msg::Twist & cmd) const
{
  return std::abs(cmd.linear.x) > config_.avoid_nonzero_epsilon ||
         std::abs(cmd.angular.z) > config_.avoid_nonzero_epsilon;
}

bool ArbiterRuntime::avoid_valid(const rclcpp::Time & now_time)
{
  if ((now_time - last_avoid_time_).seconds() > config_.avoid_cmd_timeout) {
    latest_avoid_cmd_ = geometry_msgs::msg::Twist();
    avoid_latched_ = false;
    return false;
  }

  if (!avoid_latched_) {
    return false;
  }

  if (!is_nonzero(latest_avoid_cmd_) && avoid_zero_count_ >= config_.avoid_exit_threshold &&
      (now_time - avoid_zero_start_).seconds() >= config_.avoid_exit_hysteresis_time)
  {
    avoid_latched_ = false;
    return false;
  }

  return avoid_latched_;
}

}  // namespace smart_follower_control
