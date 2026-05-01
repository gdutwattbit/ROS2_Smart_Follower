#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/arbiter_runtime.hpp"

namespace
{
smart_follower_msgs::msg::PersonPoseArray make_locked_pose(const rclcpp::Time & stamp)
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  msg.header.stamp = stamp;
  msg.lock_id = 7;
  msg.lock_state = smart_follower_msgs::msg::PersonPoseArray::LOCKED;

  smart_follower_msgs::msg::TrackedPerson person;
  person.track_id = 7;
  person.track_state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
  person.position.x = 1.0;
  person.position.y = 0.2;
  msg.persons.push_back(person);
  return msg;
}
}  // namespace

TEST(ArbiterRuntime, FollowsWithoutAgeBasedDegradation)
{
  smart_follower_control::ArbiterRuntime runtime;
  smart_follower_control::ArbiterRuntimeConfig config;
  runtime.set_config(config);
  runtime.activate();

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_person_pose(make_locked_pose(t0));

  geometry_msgs::msg::Twist follow;
  follow.linear.x = 0.4;
  follow.angular.z = 0.1;
  runtime.on_follow_cmd(follow);

  auto out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(0.1));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.1);

  out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(0.3));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.1);

  out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(2.5));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.1);

  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(2.5));
  EXPECT_EQ(snapshot.mode, smart_follower_control::ArbiterMode::FOLLOW);
  EXPECT_FALSE(snapshot.stop_latched);
}

TEST(ArbiterRuntime, HandlesStopAndResetExplicitly)
{
  smart_follower_control::ArbiterRuntime runtime;
  smart_follower_control::ArbiterRuntimeConfig config;
  runtime.set_config(config);
  runtime.activate();

  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_person_pose(make_locked_pose(t0));

  geometry_msgs::msg::Twist follow;
  follow.linear.x = 0.4;
  follow.angular.z = 0.1;
  runtime.on_follow_cmd(follow);

  smart_follower_msgs::msg::FollowCommand stop;
  stop.command = smart_follower_msgs::msg::FollowCommand::ESTOP;
  runtime.on_user_cmd(stop);

  auto out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(0.1));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.0);

  smart_follower_msgs::msg::FollowCommand reset;
  reset.command = smart_follower_msgs::msg::FollowCommand::RESET;
  runtime.on_user_cmd(reset);
  runtime.on_follow_cmd(follow);

  out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(0.2));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.1);

  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.2));
  EXPECT_EQ(snapshot.mode, smart_follower_control::ArbiterMode::FOLLOW);
  EXPECT_FALSE(snapshot.stop_latched);
}

TEST(ArbiterRuntime, SwitchesToAvoidWhenAvoidanceIsLatched)
{
  smart_follower_control::ArbiterRuntime runtime;
  smart_follower_control::ArbiterRuntimeConfig config;
  config.avoid_enter_threshold = 2;
  config.avoid_exit_threshold = 2;
  runtime.set_config(config);
  runtime.activate();

  geometry_msgs::msg::Twist follow;
  follow.linear.x = 0.3;
  follow.angular.z = 0.1;
  runtime.on_follow_cmd(follow);

  geometry_msgs::msg::Twist avoid;
  avoid.angular.z = 0.5;
  const rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  runtime.on_avoid_cmd(avoid, t0);
  runtime.on_avoid_cmd(avoid, t0 + rclcpp::Duration::from_seconds(0.01));

  const auto out = runtime.compute_output(t0 + rclcpp::Duration::from_seconds(0.02));
  EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.angular.z, 0.5);

  const auto snapshot = runtime.snapshot(t0 + rclcpp::Duration::from_seconds(0.02));
  EXPECT_EQ(snapshot.mode, smart_follower_control::ArbiterMode::AVOID);
  EXPECT_TRUE(snapshot.avoid_latched);
}
