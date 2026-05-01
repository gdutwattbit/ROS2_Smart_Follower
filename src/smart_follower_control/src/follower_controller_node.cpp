#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/control_node_common.hpp"
#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/follower_runtime.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"

namespace smart_follower_control
{

namespace
{
void declare_follower_parameters(rclcpp_lifecycle::LifecycleNode & node)
{
  node.declare_parameter("person_pose_topic", std::string("person_pose"));
  node.declare_parameter("cmd_vel_follow_topic", std::string("cmd_vel_follow"));
  node.declare_parameter("control_rate", 20.0);
  node.declare_parameter("target_distance", 1.0);
  node.declare_parameter("theta_deadzone", 0.03);
  node.declare_parameter("stop_hold.distance", 0.05);
  node.declare_parameter("stop_hold.angle", 0.10);
  node.declare_parameter("stop_hold.speed_mps", 0.12);
  node.declare_parameter("target_timeout", 0.4);
  node.declare_parameter("prediction_horizon_s", 0.25);
  node.declare_parameter("velocity_ema_alpha", 0.70);
  node.declare_parameter("pid_r.kp", 0.8);
  node.declare_parameter("pid_r.ki", 0.0);
  node.declare_parameter("pid_r.kd", 0.1);
  node.declare_parameter("pid_t.kp", 1.2);
  node.declare_parameter("pid_t.ki", 0.0);
  node.declare_parameter("pid_t.kd", 0.1);
  node.declare_parameter("pid_i_limit", 0.5);
  node.declare_parameter("pid_kaw", 0.2);
  node.declare_parameter("limits.v_max", 0.6);
  node.declare_parameter("limits.w_max", 1.2);
  node.declare_parameter("limits.dv_max", 0.5);
  node.declare_parameter("limits.dw_max", 1.5);
}

void normalize_follower_runtime(FollowerRuntimeConfig & config)
{
  config.control_rate = clamp_rate_hz(config.control_rate);
  config.theta_deadzone = clamp_non_negative(config.theta_deadzone);
  config.stop_hold_distance = clamp_non_negative(config.stop_hold_distance);
  config.stop_hold_angle = clamp_non_negative(config.stop_hold_angle);
  config.stop_hold_speed_mps = clamp_non_negative(config.stop_hold_speed_mps);
  config.target_timeout = clamp_non_negative(config.target_timeout);
  config.prediction_horizon_s = clamp_non_negative(config.prediction_horizon_s);
  config.velocity_ema_alpha = std::clamp(config.velocity_ema_alpha, 0.0, 1.0);
  config.i_limit = clamp_non_negative(config.i_limit);
  config.v_max = clamp_non_negative(config.v_max);
  config.w_max = clamp_non_negative(config.w_max);
  config.dv_max = clamp_non_negative(config.dv_max);
  config.dw_max = clamp_non_negative(config.dw_max);
}
}  // namespace

class FollowerControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowerControllerNode()
  : rclcpp_lifecycle::LifecycleNode("follower_controller_node")
  {
    declare_follower_parameters(*this);
  }

private:
  struct Params
  {
    std::string person_pose_topic{"person_pose"};
    std::string cmd_vel_follow_topic{"cmd_vel_follow"};
    FollowerRuntimeConfig runtime;
  } p_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};
  FollowerRuntime runtime_;

  void apply_runtime_config()
  {
    runtime_.set_config(p_.runtime);
  }

  bool needs_interface_recreation(const Params & next) const
  {
    return next.person_pose_topic != p_.person_pose_topic ||
           next.cmd_vel_follow_topic != p_.cmd_vel_follow_topic ||
           next.runtime.control_rate != p_.runtime.control_rate;
  }

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.runtime.control_rate = get_parameter("control_rate").as_double();
    p_.runtime.target_distance = get_parameter("target_distance").as_double();
    p_.runtime.theta_deadzone = get_parameter("theta_deadzone").as_double();
    p_.runtime.stop_hold_distance = get_parameter("stop_hold.distance").as_double();
    p_.runtime.stop_hold_angle = get_parameter("stop_hold.angle").as_double();
    p_.runtime.stop_hold_speed_mps = get_parameter("stop_hold.speed_mps").as_double();
    p_.runtime.target_timeout = get_parameter("target_timeout").as_double();
    p_.runtime.prediction_horizon_s = get_parameter("prediction_horizon_s").as_double();
    p_.runtime.velocity_ema_alpha = get_parameter("velocity_ema_alpha").as_double();
    p_.runtime.kp_r = get_parameter("pid_r.kp").as_double();
    p_.runtime.ki_r = get_parameter("pid_r.ki").as_double();
    p_.runtime.kd_r = get_parameter("pid_r.kd").as_double();
    p_.runtime.kp_t = get_parameter("pid_t.kp").as_double();
    p_.runtime.ki_t = get_parameter("pid_t.ki").as_double();
    p_.runtime.kd_t = get_parameter("pid_t.kd").as_double();
    p_.runtime.i_limit = get_parameter("pid_i_limit").as_double();
    p_.runtime.kaw = get_parameter("pid_kaw").as_double();
    p_.runtime.v_max = get_parameter("limits.v_max").as_double();
    p_.runtime.w_max = get_parameter("limits.w_max").as_double();
    p_.runtime.dv_max = get_parameter("limits.dv_max").as_double();
    p_.runtime.dw_max = get_parameter("limits.dw_max").as_double();
    normalize_follower_runtime(p_.runtime);
    apply_runtime_config();
  }

  void on_pose_message(const smart_follower_msgs::msg::PersonPoseArray::SharedPtr msg)
  {
    if (msg) {
      runtime_.on_pose(*msg);
    }
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = begin_recreate_lifecycle_publisher(
      cmd_pub_, preserve_activation, [this]() { publish_zero(); });

    timer_.reset();
    pose_sub_.reset();
    cmd_pub_.reset();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_follow_topic, 10);
    pose_sub_ = create_subscription<smart_follower_msgs::msg::PersonPoseArray>(
      p_.person_pose_topic,
      10,
      std::bind(&FollowerControllerNode::on_pose_message, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      hz_to_period(p_.runtime.control_rate), std::bind(&FollowerControllerNode::on_timer, this));

    restore_lifecycle_publisher(cmd_pub_, was_active);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    recreate_interfaces(false);

    diagnostics_.setHardwareID("smart_follower_controller");
    diagnostics_.add("follow_controller", this, &FollowerControllerNode::diag_callback);
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FollowerControllerNode::on_parameters_set, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    activate_publisher(cmd_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    publish_zero();
    deactivate_publisher(cmd_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pose_sub_.reset();
    cmd_pub_.reset();
    param_callback_handle_.reset();
    runtime_.clear();
    return CallbackReturn::SUCCESS;
  }

  void apply_parameter_override(Params & target, const rclcpp::Parameter & param)
  {
    const auto & name = param.get_name();
    if (name == "person_pose_topic") target.person_pose_topic = param.as_string();
    else if (name == "cmd_vel_follow_topic") target.cmd_vel_follow_topic = param.as_string();
    else if (name == "control_rate") target.runtime.control_rate = param.as_double();
    else if (name == "target_distance") target.runtime.target_distance = param.as_double();
    else if (name == "theta_deadzone") target.runtime.theta_deadzone = param.as_double();
    else if (name == "stop_hold.distance") target.runtime.stop_hold_distance = param.as_double();
    else if (name == "stop_hold.angle") target.runtime.stop_hold_angle = param.as_double();
    else if (name == "stop_hold.speed_mps") target.runtime.stop_hold_speed_mps = param.as_double();
    else if (name == "target_timeout") target.runtime.target_timeout = param.as_double();
    else if (name == "prediction_horizon_s") target.runtime.prediction_horizon_s = param.as_double();
    else if (name == "velocity_ema_alpha") target.runtime.velocity_ema_alpha = param.as_double();
    else if (name == "pid_r.kp") target.runtime.kp_r = param.as_double();
    else if (name == "pid_r.ki") target.runtime.ki_r = param.as_double();
    else if (name == "pid_r.kd") target.runtime.kd_r = param.as_double();
    else if (name == "pid_t.kp") target.runtime.kp_t = param.as_double();
    else if (name == "pid_t.ki") target.runtime.ki_t = param.as_double();
    else if (name == "pid_t.kd") target.runtime.kd_t = param.as_double();
    else if (name == "pid_i_limit") target.runtime.i_limit = param.as_double();
    else if (name == "pid_kaw") target.runtime.kaw = param.as_double();
    else if (name == "limits.v_max") target.runtime.v_max = param.as_double();
    else if (name == "limits.w_max") target.runtime.w_max = param.as_double();
    else if (name == "limits.dv_max") target.runtime.dv_max = param.as_double();
    else if (name == "limits.dw_max") target.runtime.dw_max = param.as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    normalize_follower_runtime(candidate.runtime);
    const bool recreate = needs_interface_recreation(candidate);
    p_ = candidate;
    apply_runtime_config();
    if (recreate) {
      recreate_interfaces(is_primary_active(*this));
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] follower controller parameters hot-reloaded.",
      kRuntimeVersion);

    return make_ok_result();
  }

  void on_timer()
  {
    if (!is_primary_active(*this)) {
      return;
    }

    const auto now_time = now();
    publish_if_activated(cmd_pub_, runtime_.compute_command(now_time));

    const auto snapshot = runtime_.snapshot(now_time);
    if (snapshot.target_seen && !snapshot.target_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        800,
        "[%s] follower target invalid: reason=%s age=%.3f timeout=%.3f pred_age=%.3f last_cmd=(%.3f,%.3f) target_v=(%.3f,%.3f) speed=%.3f invalid_events=%zu pose_lock=(id:%d state:%d found:%d confirmed:%d finite:%d)",
        kRuntimeVersion,
        snapshot.target_invalid_reason.empty() ? "unknown" : snapshot.target_invalid_reason.c_str(),
        snapshot.target_age_s,
        p_.runtime.target_timeout,
        snapshot.prediction_age_s,
        snapshot.last_cmd_v,
        snapshot.last_cmd_w,
        snapshot.target_vx,
        snapshot.target_vy,
        snapshot.target_speed_mps,
        snapshot.invalid_event_count,
        snapshot.last_pose_lock_id,
        snapshot.last_pose_lock_state,
        snapshot.locked_track_found ? 1 : 0,
        snapshot.locked_track_confirmed ? 1 : 0,
        snapshot.locked_track_finite ? 1 : 0);
    }

    diagnostics_.force_update();
  }

  void publish_zero()
  {
    runtime_.reset_output();
    publish_if_activated(cmd_pub_, geometry_msgs::msg::Twist());
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto snapshot = runtime_.snapshot(now());
    stat.add("last_cmd_v", snapshot.last_cmd_v);
    stat.add("last_cmd_w", snapshot.last_cmd_w);
    stat.add("target_valid", snapshot.target_valid);
    stat.add("target_seen", snapshot.target_seen);
    stat.add("target_age_s", snapshot.target_age_s);
    stat.add("target_vx", snapshot.target_vx);
    stat.add("target_vy", snapshot.target_vy);
    stat.add("target_speed_mps", snapshot.target_speed_mps);
    stat.add("prediction_age_s", snapshot.prediction_age_s);
    stat.add("predicted_target_valid", snapshot.predicted_target_valid);
    stat.add("stale_target_hold", snapshot.stale_target_hold);
    stat.add("hold_zone_active", snapshot.hold_zone_active);
    stat.add("raw_theta", snapshot.raw_theta);
    stat.add("last_pose_lock_id", snapshot.last_pose_lock_id);
    stat.add("last_pose_lock_state", snapshot.last_pose_lock_state);
    stat.add("locked_track_found", snapshot.locked_track_found);
    stat.add("locked_track_confirmed", snapshot.locked_track_confirmed);
    stat.add("locked_track_finite", snapshot.locked_track_finite);
    stat.add("target_invalid_reason", snapshot.target_invalid_reason);
    stat.add("invalid_event_count", static_cast<int>(snapshot.invalid_event_count));
    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Follower control active";
    if (!snapshot.target_seen) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Waiting for target input";
    } else if (!snapshot.target_valid) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = snapshot.target_invalid_reason.empty() ? "Target invalid" : (std::string("Target invalid: ") + snapshot.target_invalid_reason);
    }
    stat.summary(level, message);
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::FollowerControllerNode>(argc, argv);
}




