#include <cmath>

#include <gtest/gtest.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/follower_runtime.hpp"

namespace
{
smart_follower_msgs::msg::PersonPoseArray make_locked_pose(const rclcpp::Time & stamp, double x, double y)
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  msg.header.stamp = stamp;
  msg.lock_id = 3;
  msg.lock_state = smart_follower_msgs::msg::PersonPoseArray::LOCKED;

  smart_follower_msgs::msg::TrackedPerson person;
  person.track_id = 3;
  person.track_state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
  person.position.x = x;
  person.position.y = y;
  msg.persons.push_back(person);
  return msg;
}

smart_follower_msgs::msg::PersonPoseArray make_unlocked_pose(const rclcpp::Time & stamp)
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  msg.header.stamp = stamp;
  msg.lock_id = -1;
  msg.lock_state = smart_follower_msgs::msg::PersonPoseArray::IDLE;
  return msg;
}

smart_follower_control::FollowerRuntimeConfig make_test_config()
{
  smart_follower_control::FollowerRuntimeConfig config;
  config.control_rate = 20.0;
  config.target_timeout = 0.3;
  config.prediction_horizon_s = 0.25;
  config.velocity_ema_alpha = 0.0;
  config.max_target_speed_mps = 10.0;
  config.target_distance = 0.0;
  config.theta_deadzone = 0.0;
  config.kp_r = 1.0;
  config.ki_r = 0.0;
  config.kd_r = 0.0;
  config.kp_t = 0.0;
  config.ki_t = 0.0;
  config.kd_t = 0.0;
  config.v_max = 10.0;
  config.w_max = 10.0;
  config.dv_max = 100.0;
  config.dw_max = 100.0;
  return config;
}
}  // namespace

TEST(FollowerRuntime, OutputsZeroAfterTimeout)
{
  smart_follower_control::FollowerRuntime runtime;
  smart_follower_control::FollowerRuntimeConfig config;
  config.control_rate = 20.0;
  config.target_timeout = 0.3;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.4, 0.2));

  auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.1));
  EXPECT_TRUE(std::isfinite(cmd.linear.x));
  EXPECT_TRUE(std::isfinite(cmd.angular.z));

  cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.5));
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(FollowerRuntime, PredictsBetweenPoseUpdates)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.15));
  EXPECT_NEAR(cmd.linear.x, 1.3, 1e-3);
  EXPECT_NEAR(cmd.angular.z, 0.0, 1e-6);
}

TEST(FollowerRuntime, FirstTargetDoesNotExtrapolate)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.4, 0.0));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.1));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.1));
  EXPECT_NEAR(cmd.linear.x, 1.4, 1e-3);
  EXPECT_DOUBLE_EQ(snapshot.target_vx, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.target_vy, 0.0);
}

TEST(FollowerRuntime, VelocityEstimateUsesEma)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.velocity_ema_alpha = 0.5;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));
  auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.1));
  EXPECT_NEAR(snapshot.target_vx, 1.0, 1e-6);

  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.2), 1.5, 0.0));
  snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.2));
  EXPECT_NEAR(snapshot.target_vx, 2.0, 1e-6);
}

TEST(FollowerRuntime, PredictionUsesConfiguredHorizon)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.prediction_horizon_s = 0.05;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.3));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.3));
  EXPECT_NEAR(cmd.linear.x, 1.3, 1e-3);
  EXPECT_NEAR(snapshot.prediction_age_s, 0.05, 1e-6);
}

TEST(FollowerRuntime, StopsPredictingWhenLockIsLost)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));
  runtime.on_pose(make_unlocked_pose(t0 + rclcpp::Duration::from_seconds(0.15)));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.2));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.2));
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
  EXPECT_FALSE(snapshot.target_valid);
  EXPECT_FALSE(snapshot.predicted_target_valid);
}
