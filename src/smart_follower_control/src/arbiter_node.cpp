#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/arbiter_runtime.hpp"
#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"

namespace smart_follower_control
{

class ArbiterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ArbiterNode()
  : rclcpp_lifecycle::LifecycleNode("arbiter_node")
  {
    declare_parameter("person_pose_topic", std::string("person_pose"));
    declare_parameter("cmd_vel_follow_topic", std::string("cmd_vel_follow"));
    declare_parameter("cmd_vel_avoid_topic", std::string("cmd_vel_avoid"));
    declare_parameter("follow_command_topic", std::string("follow_command"));
    declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));

    declare_parameter("publish_rate", 20.0);
    declare_parameter("lost_time_normal_max", 0.2);
    declare_parameter("lost_time_degraded_max", 0.6);
    declare_parameter("lost_time_search_max", 2.0);
    declare_parameter("degraded_linear_scale", 0.5);
    declare_parameter("search_angular_speed", 0.3);

    declare_parameter("avoid_enter_threshold", 3);
    declare_parameter("avoid_exit_threshold", 5);
    declare_parameter("avoid_exit_hysteresis_time", 0.2);
    declare_parameter("avoid_cmd_timeout", 0.2);
    declare_parameter("avoid_nonzero_epsilon", 1e-3);
  }

private:
  struct Params
  {
    std::string person_pose_topic{"person_pose"};
    std::string cmd_vel_follow_topic{"cmd_vel_follow"};
    std::string cmd_vel_avoid_topic{"cmd_vel_avoid"};
    std::string follow_command_topic{"follow_command"};
    std::string cmd_vel_topic{"/cmd_vel"};
    double publish_rate{20.0};
    ArbiterRuntimeConfig runtime;
  } p_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr avoid_sub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};
  ArbiterRuntime runtime_;

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.cmd_vel_avoid_topic = get_parameter("cmd_vel_avoid_topic").as_string();
    p_.follow_command_topic = get_parameter("follow_command_topic").as_string();
    p_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    p_.publish_rate = get_parameter("publish_rate").as_double();

    p_.runtime.thresholds.lost_time_normal_max = get_parameter("lost_time_normal_max").as_double();
    p_.runtime.thresholds.lost_time_degraded_max = get_parameter("lost_time_degraded_max").as_double();
    p_.runtime.thresholds.lost_time_search_max = get_parameter("lost_time_search_max").as_double();
    p_.runtime.degraded_linear_scale = get_parameter("degraded_linear_scale").as_double();
    p_.runtime.search_angular_speed = get_parameter("search_angular_speed").as_double();
    p_.runtime.avoid_enter_threshold = get_parameter("avoid_enter_threshold").as_int();
    p_.runtime.avoid_exit_threshold = get_parameter("avoid_exit_threshold").as_int();
    p_.runtime.avoid_exit_hysteresis_time = get_parameter("avoid_exit_hysteresis_time").as_double();
    p_.runtime.avoid_cmd_timeout = get_parameter("avoid_cmd_timeout").as_double();
    p_.runtime.avoid_nonzero_epsilon = get_parameter("avoid_nonzero_epsilon").as_double();

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
    follow_sub_.reset();
    avoid_sub_.reset();
    cmd_sub_.reset();
    cmd_pub_.reset();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_topic, 10);
    pose_sub_ = create_subscription<smart_follower_msgs::msg::PersonPoseArray>(
      p_.person_pose_topic,
      10,
      [this](const smart_follower_msgs::msg::PersonPoseArray::SharedPtr msg) {
        if (msg) {
          runtime_.on_person_pose(*msg);
        }
      });
    follow_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_follow_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          runtime_.on_follow_cmd(*msg);
        }
      });
    avoid_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_avoid_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          runtime_.on_avoid_cmd(*msg, now());
        }
      });
    cmd_sub_ = create_subscription<smart_follower_msgs::msg::FollowCommand>(
      p_.follow_command_topic,
      10,
      [this](const smart_follower_msgs::msg::FollowCommand::SharedPtr msg) {
        if (msg) {
          runtime_.on_user_cmd(*msg);
        }
      });

    timer_ = create_wall_timer(hz_to_period(p_.publish_rate), std::bind(&ArbiterNode::on_timer, this));

    if (was_active) {
      activate_publisher(cmd_pub_);
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    recreate_interfaces(false);

    diagnostics_.setHardwareID("smart_follower_arbiter");
    diagnostics_.add("arbiter_status", this, &ArbiterNode::diag_callback);
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArbiterNode::on_parameters_set, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    activate_publisher(cmd_pub_);
    runtime_.activate();
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
    follow_sub_.reset();
    avoid_sub_.reset();
    cmd_sub_.reset();
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
    else if (name == "cmd_vel_avoid_topic") target.cmd_vel_avoid_topic = param.as_string();
    else if (name == "follow_command_topic") target.follow_command_topic = param.as_string();
    else if (name == "cmd_vel_topic") target.cmd_vel_topic = param.as_string();
    else if (name == "publish_rate") target.publish_rate = param.as_double();
    else if (name == "lost_time_normal_max") target.runtime.thresholds.lost_time_normal_max = param.as_double();
    else if (name == "lost_time_degraded_max") target.runtime.thresholds.lost_time_degraded_max = param.as_double();
    else if (name == "lost_time_search_max") target.runtime.thresholds.lost_time_search_max = param.as_double();
    else if (name == "degraded_linear_scale") target.runtime.degraded_linear_scale = param.as_double();
    else if (name == "search_angular_speed") target.runtime.search_angular_speed = param.as_double();
    else if (name == "avoid_enter_threshold") target.runtime.avoid_enter_threshold = param.as_int();
    else if (name == "avoid_exit_threshold") target.runtime.avoid_exit_threshold = param.as_int();
    else if (name == "avoid_exit_hysteresis_time") target.runtime.avoid_exit_hysteresis_time = param.as_double();
    else if (name == "avoid_cmd_timeout") target.runtime.avoid_cmd_timeout = param.as_double();
    else if (name == "avoid_nonzero_epsilon") target.runtime.avoid_nonzero_epsilon = param.as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    candidate.publish_rate = std::max(1.0, candidate.publish_rate);
    candidate.runtime.thresholds.lost_time_normal_max = std::max(0.0, candidate.runtime.thresholds.lost_time_normal_max);
    candidate.runtime.thresholds.lost_time_degraded_max = std::max(
      candidate.runtime.thresholds.lost_time_normal_max,
      candidate.runtime.thresholds.lost_time_degraded_max);
    candidate.runtime.thresholds.lost_time_search_max = std::max(
      candidate.runtime.thresholds.lost_time_degraded_max,
      candidate.runtime.thresholds.lost_time_search_max);
    candidate.runtime.degraded_linear_scale = std::clamp(candidate.runtime.degraded_linear_scale, 0.0, 1.0);
    candidate.runtime.avoid_enter_threshold = std::max(1, candidate.runtime.avoid_enter_threshold);
    candidate.runtime.avoid_exit_threshold = std::max(1, candidate.runtime.avoid_exit_threshold);
    candidate.runtime.avoid_exit_hysteresis_time = std::max(0.0, candidate.runtime.avoid_exit_hysteresis_time);
    candidate.runtime.avoid_cmd_timeout = std::max(0.0, candidate.runtime.avoid_cmd_timeout);
    candidate.runtime.avoid_nonzero_epsilon = std::max(0.0, candidate.runtime.avoid_nonzero_epsilon);

    p_ = candidate;
    runtime_.set_config(p_.runtime);
    recreate_interfaces(is_primary_active(*this));

    RCLCPP_INFO(
      get_logger(),
      "[%s] arbiter parameters hot-reloaded: pose=%s follow=%s avoid=%s cmd=%s rate=%.2f",
      kRuntimeVersion,
      p_.person_pose_topic.c_str(),
      p_.cmd_vel_follow_topic.c_str(),
      p_.cmd_vel_avoid_topic.c_str(),
      p_.cmd_vel_topic.c_str(),
      p_.publish_rate);

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

    publish_if_activated(cmd_pub_, runtime_.compute_output(now()));
    diagnostics_.force_update();
  }

  void publish_zero()
  {
    publish_if_activated(cmd_pub_, geometry_msgs::msg::Twist());
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto snapshot = runtime_.snapshot(now());
    stat.add("mode", static_cast<int>(snapshot.mode));
    stat.add("stop_latched", snapshot.stop_latched);
    stat.add("avoid_latched", snapshot.avoid_latched);
    stat.add("last_target_age_s", snapshot.last_target_age_s);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Arbiter running");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::ArbiterNode>(argc, argv);
}
