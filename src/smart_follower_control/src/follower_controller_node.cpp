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

#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/follower_runtime.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"

namespace smart_follower_control
{

class FollowerControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowerControllerNode()
  : rclcpp_lifecycle::LifecycleNode("follower_controller_node")
  {
    declare_parameter("person_pose_topic", std::string("person_pose"));
    declare_parameter("cmd_vel_follow_topic", std::string("cmd_vel_follow"));
    declare_parameter("control_rate", 20.0);
    declare_parameter("target_distance", 1.0);
    declare_parameter("theta_deadzone", 0.03);
    declare_parameter("target_timeout", 0.3);
    declare_parameter("pid_r.kp", 0.8);
    declare_parameter("pid_r.ki", 0.0);
    declare_parameter("pid_r.kd", 0.1);
    declare_parameter("pid_t.kp", 1.2);
    declare_parameter("pid_t.ki", 0.0);
    declare_parameter("pid_t.kd", 0.1);
    declare_parameter("pid_i_limit", 0.5);
    declare_parameter("pid_kaw", 0.2);
    declare_parameter("limits.v_max", 0.6);
    declare_parameter("limits.w_max", 1.2);
    declare_parameter("limits.dv_max", 0.5);
    declare_parameter("limits.dw_max", 1.5);
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

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.runtime.control_rate = get_parameter("control_rate").as_double();
    p_.runtime.target_distance = get_parameter("target_distance").as_double();
    p_.runtime.theta_deadzone = get_parameter("theta_deadzone").as_double();
    p_.runtime.target_timeout = get_parameter("target_timeout").as_double();
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
    runtime_.set_config(p_.runtime);
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation && publisher_is_activated(cmd_pub_);
    if (was_active) {
      publish_zero();
      deactivate_publisher(cmd_pub_);
    }

    timer_.reset();
    pose_sub_.reset();
    cmd_pub_.reset();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_follow_topic, 10);
    pose_sub_ = create_subscription<smart_follower_msgs::msg::PersonPoseArray>(
      p_.person_pose_topic,
      10,
      [this](const smart_follower_msgs::msg::PersonPoseArray::SharedPtr msg) {
        if (msg) {
          runtime_.on_pose(*msg);
        }
      });

    timer_ = create_wall_timer(hz_to_period(p_.runtime.control_rate), std::bind(&FollowerControllerNode::on_timer, this));

    if (was_active) {
      activate_publisher(cmd_pub_);
    }
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
    else if (name == "target_timeout") target.runtime.target_timeout = param.as_double();
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

    candidate.runtime.control_rate = std::max(1.0, candidate.runtime.control_rate);
    candidate.runtime.target_timeout = std::max(0.0, candidate.runtime.target_timeout);
    candidate.runtime.theta_deadzone = std::max(0.0, candidate.runtime.theta_deadzone);
    candidate.runtime.i_limit = std::max(0.0, candidate.runtime.i_limit);
    candidate.runtime.v_max = std::max(0.0, candidate.runtime.v_max);
    candidate.runtime.w_max = std::max(0.0, candidate.runtime.w_max);
    candidate.runtime.dv_max = std::max(0.0, candidate.runtime.dv_max);
    candidate.runtime.dw_max = std::max(0.0, candidate.runtime.dw_max);

    p_ = candidate;
    runtime_.set_config(p_.runtime);
    recreate_interfaces(is_primary_active(*this));

    RCLCPP_INFO(
      get_logger(),
      "[%s] controller parameters hot-reloaded: pose=%s cmd=%s rate=%.2f timeout=%.2f",
      kRuntimeVersion,
      p_.person_pose_topic.c_str(),
      p_.cmd_vel_follow_topic.c_str(),
      p_.runtime.control_rate,
      p_.runtime.target_timeout);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void on_timer()
  {
    if (!is_primary_active(*this)) {
      return;
    }

    publish_if_activated(cmd_pub_, runtime_.compute_command(now()));
    diagnostics_.force_update();
  }

  void publish_zero()
  {
    runtime_.reset_output();
    publish_if_activated(cmd_pub_, geometry_msgs::msg::Twist());
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto snapshot = runtime_.snapshot();
    stat.add("last_cmd_v", snapshot.last_cmd_v);
    stat.add("last_cmd_w", snapshot.last_cmd_w);
    stat.add("target_valid", snapshot.target_valid);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Follower control active");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::FollowerControllerNode>(argc, argv);
}
