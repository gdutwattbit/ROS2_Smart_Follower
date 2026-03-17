#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include "smart_follower_control/obstacle_runtime.hpp"

TEST(ObstacleRuntime, TurnsAwayFromCloserLeftObstacle)
{
  smart_follower_control::ObstacleRuntime runtime;
  smart_follower_control::ObstacleRuntimeConfig config;
  config.turn_speed = 0.6;
  config.slow_turn_speed = 0.3;
  runtime.set_config(config);
  runtime.clear();

  const rclcpp::Time now_time(20, 0, RCL_ROS_TIME);
  runtime.on_left_range(0.15, now_time);
  runtime.on_right_range(1.0, now_time);

  geometry_msgs::msg::Twist cmd = runtime.compute_command(now_time);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_LT(cmd.angular.z, 0.0);
}

TEST(ObstacleRuntime, IgnoresStaleRanges)
{
  smart_follower_control::ObstacleRuntime runtime;
  runtime.clear();

  const rclcpp::Time t0(30, 0, RCL_ROS_TIME);
  runtime.on_left_range(0.1, t0);
  runtime.on_right_range(0.1, t0);

  geometry_msgs::msg::Twist cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.8));
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}