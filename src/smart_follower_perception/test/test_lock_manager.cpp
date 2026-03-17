#include <gtest/gtest.h>

#include <unordered_map>

#include <opencv2/core.hpp>

#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_perception/lock_manager.hpp"

using smart_follower_perception::LockConfig;
using smart_follower_perception::LockManager;
using smart_follower_perception::Track;

namespace
{

smart_follower_msgs::msg::FollowCommand make_cmd(uint8_t cmd, int32_t target_id)
{
  smart_follower_msgs::msg::FollowCommand msg;
  msg.command = cmd;
  msg.target_id = target_id;
  return msg;
}

Track make_confirmed_track(int id, const cv::Rect2f & bbox, float conf)
{
  Track t;
  t.id = id;
  t.state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
  t.bbox = bbox;
  t.confidence = conf;
  return t;
}

}  // namespace

TEST(LockManager, StableLockAfterNFrames)
{
  LockManager lock;
  LockConfig cfg;
  cfg.stable_frames = 2;
  cfg.hold_sec = 0.6F;
  cfg.switch_sec = 2.0F;
  cfg.center_roi_ratio = 0.8F;
  cfg.target_area_ratio = 0.04F;
  lock.configure(cfg);

  const cv::Size image_size(640, 480);

  std::unordered_map<int, Track> tracks;
  tracks.emplace(1, make_confirmed_track(1, cv::Rect2f(300, 200, 80, 160), 0.9F));

  auto cmd = make_cmd(smart_follower_msgs::msg::FollowCommand::LOCK, -1);
  lock.handle_command(cmd, rclcpp::Time(1, 0, RCL_ROS_TIME));

  lock.update(tracks, image_size, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(lock.lock_id(), -1);

  lock.update(tracks, image_size, rclcpp::Time(1, 10000000, RCL_ROS_TIME));
  EXPECT_EQ(lock.lock_id(), 1);
  EXPECT_EQ(lock.lock_state(), smart_follower_msgs::msg::PersonPoseArray::LOCKED);
}

TEST(LockManager, BecomesLostAfterHoldTime)
{
  LockManager lock;
  LockConfig cfg;
  cfg.stable_frames = 1;
  cfg.hold_sec = 0.1F;
  cfg.switch_sec = 2.0F;
  cfg.center_roi_ratio = 1.0F;
  cfg.target_area_ratio = 0.04F;
  lock.configure(cfg);

  const cv::Size image_size(640, 480);

  std::unordered_map<int, Track> tracks;
  tracks.emplace(1, make_confirmed_track(1, cv::Rect2f(300, 200, 80, 160), 0.9F));

  auto cmd = make_cmd(smart_follower_msgs::msg::FollowCommand::LOCK, -1);
  lock.handle_command(cmd, rclcpp::Time(1, 0, RCL_ROS_TIME));
  lock.update(tracks, image_size, rclcpp::Time(1, 0, RCL_ROS_TIME));

  std::unordered_map<int, Track> empty;
  lock.update(empty, image_size, rclcpp::Time(1, 200000000, RCL_ROS_TIME));
  EXPECT_EQ(lock.lock_state(), smart_follower_msgs::msg::PersonPoseArray::LOST);
}
