#pragma once

#include <limits>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>

namespace smart_follower_control
{

struct ObstacleRuntimeConfig
{
  double d_min{0.12};
  double t_react{0.20};
  double a_brake{0.8};
  double margin{0.08};
  double exit_margin{0.08};
  double turn_speed{0.5};
  double slow_turn_speed{0.25};
  double back_speed{-0.15};
};

struct ObstacleRuntimeSnapshot
{
  double left_dist{0.0};
  double right_dist{0.0};
  double left_age_s{-1.0};
  double right_age_s{-1.0};
  double current_speed{0.0};
};

class ObstacleRuntime
{
public:
  void set_config(const ObstacleRuntimeConfig & config);
  const ObstacleRuntimeConfig & config() const { return config_; }

  void clear();
  void on_left_range(double range, const rclcpp::Time & stamp);
  void on_right_range(double range, const rclcpp::Time & stamp);
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);

  geometry_msgs::msg::Twist compute_command(const rclcpp::Time & now_time);
  ObstacleRuntimeSnapshot snapshot(const rclcpp::Time & now_time) const;

private:
  ObstacleRuntimeConfig config_;
  double left_dist_{std::numeric_limits<double>::infinity()};
  double right_dist_{std::numeric_limits<double>::infinity()};
  double current_speed_{0.0};

  rclcpp::Time left_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace smart_follower_control
