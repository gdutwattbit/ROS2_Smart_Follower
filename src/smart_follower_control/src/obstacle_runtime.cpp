#include "smart_follower_control/obstacle_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace smart_follower_control
{

void ObstacleRuntime::set_config(const ObstacleRuntimeConfig & config)
{
  config_ = config;
}

void ObstacleRuntime::clear()
{
  left_dist_ = std::numeric_limits<double>::infinity();
  right_dist_ = std::numeric_limits<double>::infinity();
  current_speed_ = 0.0;
  left_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  right_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void ObstacleRuntime::on_left_range(double range, const rclcpp::Time & stamp)
{
  left_dist_ = range;
  left_stamp_ = stamp;
}

void ObstacleRuntime::on_right_range(double range, const rclcpp::Time & stamp)
{
  right_dist_ = range;
  right_stamp_ = stamp;
}

void ObstacleRuntime::on_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  current_speed_ = std::abs(msg.linear.x);
}

geometry_msgs::msg::Twist ObstacleRuntime::compute_command(const rclcpp::Time & now_time)
{
  const auto stale = [&](const rclcpp::Time & stamp) {
    return stamp.nanoseconds() == 0 || (now_time - stamp).seconds() > 0.5;
  };

  const double left = stale(left_stamp_) ? std::numeric_limits<double>::infinity() : left_dist_;
  const double right = stale(right_stamp_) ? std::numeric_limits<double>::infinity() : right_dist_;

  const double d_safe = config_.d_min + current_speed_ * config_.t_react +
                        (current_speed_ * current_speed_) / (2.0 * std::max(1e-3, config_.a_brake)) +
                        config_.margin;
  const double d_enter = d_safe;
  const double d_exit = d_safe + config_.exit_margin;
  const double d_danger = std::max(0.18, 0.6 * d_safe);
  const double fused_min = std::min(left, right);

  geometry_msgs::msg::Twist out;
  if (fused_min > d_exit) {
    return out;
  }
  if (left < d_danger && right < d_danger) {
    out.linear.x = config_.back_speed;
    return out;
  }
  if (left < d_danger && right > d_enter) {
    out.angular.z = -std::abs(config_.turn_speed);
    return out;
  }
  if (right < d_danger && left > d_enter) {
    out.angular.z = std::abs(config_.turn_speed);
    return out;
  }
  if (fused_min < d_enter) {
    out.angular.z = (left < right) ? -std::abs(config_.slow_turn_speed) : std::abs(config_.slow_turn_speed);
  }
  return out;
}

ObstacleRuntimeSnapshot ObstacleRuntime::snapshot(const rclcpp::Time & now_time) const
{
  ObstacleRuntimeSnapshot out;
  out.left_dist = left_dist_;
  out.right_dist = right_dist_;
  out.left_age_s = left_stamp_.nanoseconds() > 0 ? std::max(0.0, (now_time - left_stamp_).seconds()) : -1.0;
  out.right_age_s = right_stamp_.nanoseconds() > 0 ? std::max(0.0, (now_time - right_stamp_).seconds()) : -1.0;
  out.current_speed = current_speed_;
  return out;
}

}  // namespace smart_follower_control
