#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/time.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/pid_controller.hpp"

namespace smart_follower_control
{

struct FollowerRuntimeConfig
{
  double control_rate{20.0};
  double target_distance{1.0};
  double steering_kalman_process_noise{6.0};
  double steering_kalman_measurement_noise{0.02};
  double steering_kalman_initial_covariance{1.0};
  double target_timeout{0.4};
  double prediction_horizon_s{0.25};
  double velocity_ema_alpha{0.70};

  double kp_r{0.8};
  double ki_r{0.0};
  double kd_r{0.1};
  double kp_t{1.2};
  double ki_t{0.0};
  double kd_t{0.1};
  double i_limit{0.5};
  double kaw{0.2};

  double v_max{0.6};
  double w_max{1.2};
  double dv_max{0.5};
  double dw_max{1.5};
};

struct FollowerRuntimeSnapshot
{
  double last_cmd_v{0.0};
  double last_cmd_w{0.0};
  bool target_valid{false};
  bool target_seen{false};
  double target_age_s{-1.0};
  double target_vx{0.0};
  double target_vy{0.0};
  double target_speed_mps{0.0};
  double prediction_age_s{-1.0};
  bool predicted_target_valid{false};
  bool steering_filter_ready{false};
  double raw_theta{0.0};
  double filtered_theta{0.0};
  int last_pose_lock_id{-1};
  int last_pose_lock_state{0};
  bool locked_track_found{false};
  bool locked_track_confirmed{false};
  bool locked_track_finite{false};
  std::string target_invalid_reason;
  std::size_t invalid_event_count{0};
};

class FollowerRuntime
{
public:
  void set_config(const FollowerRuntimeConfig & config);
  const FollowerRuntimeConfig & config() const { return config_; }

  void clear();
  void reset_output();
  void on_pose(const smart_follower_msgs::msg::PersonPoseArray & msg);
  geometry_msgs::msg::Twist compute_command(const rclcpp::Time & now_time);
  FollowerRuntimeSnapshot snapshot(const rclcpp::Time & now_time) const;

private:
  struct TargetState
  {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct SteeringKalmanState
  {
    bool initialized{false};
    double theta{0.0};
    double theta_rate{0.0};
    double p00{1.0};
    double p01{0.0};
    double p10{0.0};
    double p11{1.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  void configure_controllers();
  std::optional<TargetState> predict_target(const rclcpp::Time & now_time);
  double rate_limit(double target, double current, double accel_limit, double dt) const;
  void reset_steering_filter();
  double filter_theta_measurement(double theta, const rclcpp::Time & stamp);

  FollowerRuntimeConfig config_;
  TargetState last_target_;
  PID pid_r_;
  PID pid_t_;
  SteeringKalmanState steering_filter_;
  geometry_msgs::msg::Twist last_cmd_;
  double last_raw_theta_{0.0};
  double last_filtered_theta_{0.0};
  int last_pose_lock_id_{-1};
  int last_pose_lock_state_{0};
  bool last_pose_locked_track_found_{false};
  bool last_pose_locked_track_confirmed_{false};
  bool last_pose_locked_track_finite_{false};
  std::string last_invalid_reason_;
  std::size_t invalid_target_event_count_{0};
};

}  // namespace smart_follower_control
