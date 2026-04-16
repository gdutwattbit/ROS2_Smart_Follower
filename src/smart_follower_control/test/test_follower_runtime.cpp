#include <cmath>
#include <limits>

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

smart_follower_msgs::msg::PersonPoseArray make_locked_pose_with_nan(const rclcpp::Time & stamp)
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  msg.header.stamp = stamp;
  msg.lock_id = 3;
  msg.lock_state = smart_follower_msgs::msg::PersonPoseArray::LOCKED;

  smart_follower_msgs::msg::TrackedPerson person;
  person.track_id = 3;
  person.track_state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
  person.position.x = std::numeric_limits<double>::quiet_NaN();
  person.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.persons.push_back(person);
  return msg;
}

smart_follower_control::FollowerRuntimeConfig make_test_config()
{
  smart_follower_control::FollowerRuntimeConfig config;
  config.control_rate = 20.0;
  config.theta_deadzone = 0.0;
  config.stop_hold_distance = 0.0;
  config.stop_hold_angle = 0.0;
  config.stop_hold_speed_mps = 0.0;
  config.target_timeout = 0.3;
  config.prediction_horizon_s = 0.25;
  config.velocity_ema_alpha = 0.0;
  config.target_distance = 0.0;
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
  config.theta_deadzone = 0.0;
  config.stop_hold_distance = 0.0;
  config.stop_hold_angle = 0.0;
  config.stop_hold_speed_mps = 0.0;
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

TEST(FollowerRuntime, KeepsLastValidTargetAcrossShortNanGap)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.target_timeout = 0.4;
  config.prediction_horizon_s = 0.25;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));
  runtime.on_pose(make_locked_pose_with_nan(t0 + rclcpp::Duration::from_seconds(0.2)));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.25));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.25));
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_TRUE(snapshot.target_valid);
  EXPECT_TRUE(snapshot.predicted_target_valid);
  EXPECT_TRUE(snapshot.stale_target_hold);
  EXPECT_DOUBLE_EQ(snapshot.prediction_age_s, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.target_vx, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.target_vy, 0.0);
}

TEST(FollowerRuntime, KeepsFrozenTargetWhileInvalidFramesContinueArriving)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.target_timeout = 0.3;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));
  runtime.on_pose(make_locked_pose_with_nan(t0 + rclcpp::Duration::from_seconds(0.2)));
  runtime.on_pose(make_locked_pose_with_nan(t0 + rclcpp::Duration::from_seconds(0.45)));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.50));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.50));
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_TRUE(snapshot.target_valid);
  EXPECT_TRUE(snapshot.stale_target_hold);
  EXPECT_EQ(snapshot.target_invalid_reason, "locked_track_position_nan");
}

TEST(FollowerRuntime, KeepsFrozenTargetEvenAfterPoseMessagesStop)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.target_timeout = 0.3;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.0));
  runtime.on_pose(make_locked_pose(t0 + rclcpp::Duration::from_seconds(0.1), 1.2, 0.0));
  runtime.on_pose(make_locked_pose_with_nan(t0 + rclcpp::Duration::from_seconds(0.2)));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.55));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.55));
  EXPECT_GT(cmd.linear.x, 0.0);
  EXPECT_TRUE(snapshot.target_valid);
  EXPECT_TRUE(snapshot.predicted_target_valid);
  EXPECT_TRUE(snapshot.stale_target_hold);
  EXPECT_EQ(snapshot.target_invalid_reason, "locked_track_position_nan");
}
TEST(FollowerRuntime, ThetaDeadzoneZerosSmallHeading)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.theta_deadzone = 0.05;
  config.kp_t = 2.0;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.0, 0.02));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.05));
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

TEST(FollowerRuntime, StopHoldZoneKeepsRobotStillNearTarget)
{
  smart_follower_control::FollowerRuntime runtime;
  auto config = make_test_config();
  config.target_distance = 1.0;
  config.stop_hold_distance = 0.05;
  config.stop_hold_angle = 0.10;
  config.stop_hold_speed_mps = 0.12;
  config.kp_r = 4.0;
  config.kp_t = 1.0;
  runtime.set_config(config);

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_pose(make_locked_pose(t0, 1.03, 0.02));

  const auto cmd = runtime.compute_command(t0 + rclcpp::Duration::from_seconds(0.05));
  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.05));
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
  EXPECT_TRUE(snapshot.hold_zone_active);
}

