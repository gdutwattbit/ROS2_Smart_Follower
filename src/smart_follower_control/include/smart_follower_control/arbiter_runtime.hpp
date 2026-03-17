#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/arbiter_state_machine.hpp"

namespace smart_follower_control
{

struct ArbiterRuntimeConfig
{
  ArbiterThresholds thresholds;
  double degraded_linear_scale{0.5};
  double search_angular_speed{0.3};

  int avoid_enter_threshold{3};
  int avoid_exit_threshold{5};
  double avoid_exit_hysteresis_time{0.2};
  double avoid_cmd_timeout{0.2};
  double avoid_nonzero_epsilon{1e-3};
};

struct ArbiterRuntimeSnapshot
{
  ArbiterMode mode{ArbiterMode::STOP};
  bool stop_latched{false};
  bool avoid_latched{false};
  double last_target_age_s{-1.0};
};

class ArbiterRuntime
{
public:
  void set_config(const ArbiterRuntimeConfig & config);
  const ArbiterRuntimeConfig & config() const { return config_; }

  void activate();
  void clear();

  void on_person_pose(const smart_follower_msgs::msg::PersonPoseArray & msg);
  void on_follow_cmd(const geometry_msgs::msg::Twist & msg);
  void on_avoid_cmd(const geometry_msgs::msg::Twist & msg, const rclcpp::Time & now_time);
  void on_user_cmd(const smart_follower_msgs::msg::FollowCommand & msg);

  geometry_msgs::msg::Twist compute_output(const rclcpp::Time & now_time);
  ArbiterRuntimeSnapshot snapshot(const rclcpp::Time & now_time) const;

private:
  bool is_nonzero(const geometry_msgs::msg::Twist & cmd) const;
  bool avoid_valid(const rclcpp::Time & now_time);

  ArbiterRuntimeConfig config_;

  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoid_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time avoid_zero_start_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist latest_follow_cmd_;
  geometry_msgs::msg::Twist latest_avoid_cmd_;
  double last_target_theta_{0.0};

  int avoid_nonzero_count_{0};
  int avoid_zero_count_{0};
  bool avoid_latched_{false};
  bool stop_latched_{false};

  ArbiterMode mode_{ArbiterMode::STOP};
};

}  // namespace smart_follower_control
