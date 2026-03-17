#pragma once

#include <unordered_map>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_perception/tracker.hpp"

namespace smart_follower_perception
{

struct LockConfig
{
  int stable_frames{5};
  float hold_sec{0.6F};
  float switch_sec{2.0F};
  float center_roi_ratio{0.6F};
  float target_area_ratio{0.04F};
};

class LockManager
{
public:
  void configure(const LockConfig & config);
  void reset();

  bool handle_command(const smart_follower_msgs::msg::FollowCommand & command, const rclcpp::Time & current_stamp);
  void update(const std::unordered_map<int, Track> & tracks, const cv::Size & image_size, const rclcpp::Time & stamp);
  void set_lock_id(int track_id);

  int lock_id() const;
  uint8_t lock_state() const;
  const rclcpp::Time & last_lock_confirmed_time() const;

private:
  LockConfig config_;
  int lock_id_{-1};
  uint8_t lock_state_{smart_follower_msgs::msg::PersonPoseArray::IDLE};
  rclcpp::Time last_lock_confirmed_time_{0, 0, RCL_ROS_TIME};
  bool pending_lock_request_{false};
  int pending_stable_track_{-1};
  int pending_stable_count_{0};
};

}  // namespace smart_follower_perception
