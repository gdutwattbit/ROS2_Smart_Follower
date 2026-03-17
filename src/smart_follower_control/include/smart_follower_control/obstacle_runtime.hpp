#pragma once

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace smart_follower_control
{

struct ObstacleRuntimeConfig
{
  double d_min{0.12};
  double t_react{0.20};
  double a_brake{0.8};
  double margin{0.08};
  double exit_margin{0.08};
  double depth_roi_width_ratio{0.4};
  double depth_roi_height_ratio{0.35};
  double depth_percentile{0.05};
  int depth_sample_stride{2};
  double turn_speed{0.5};
  double slow_turn_speed{0.25};
  double back_speed{-0.15};
};

struct ObstacleRuntimeSnapshot
{
  double left_dist{0.0};
  double right_dist{0.0};
  double depth_dist{0.0};
  int depth_message_count{0};
  int depth_process_count{0};
  int depth_sample_stride{0};
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
  void on_depth(const sensor_msgs::msg::Image::SharedPtr & msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);

  geometry_msgs::msg::Twist compute_command(const rclcpp::Time & now_time);
  ObstacleRuntimeSnapshot snapshot() const;

private:
  void refresh_depth_from_latest();

  ObstacleRuntimeConfig config_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_msg_;
  mutable std::mutex depth_mutex_;

  double left_dist_{std::numeric_limits<double>::infinity()};
  double right_dist_{std::numeric_limits<double>::infinity()};
  double depth_dist_{std::numeric_limits<double>::infinity()};
  double current_speed_{0.0};

  rclcpp::Time left_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time depth_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t depth_message_count_{0};
  std::size_t depth_process_count_{0};
};

}  // namespace smart_follower_control
