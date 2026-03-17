#include <cmath>

#include <gtest/gtest.h>

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
