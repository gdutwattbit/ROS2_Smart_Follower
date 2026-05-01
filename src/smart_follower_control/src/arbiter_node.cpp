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
#include "smart_follower_control/control_node_common.hpp"
#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"

namespace smart_follower_control
{

namespace
{
constexpr const char * kDeprecatedArbiterParameters[] = {
  "lost_time_normal_max",
  "lost_time_degraded_max",
  "lost_time_search_max",
  "degraded_linear_scale",
  "search_angular_speed",
};

bool contains_parameter(
  const std::vector<rclcpp::Parameter> & params,
  const char * name)
{
  return std::any_of(params.begin(), params.end(), [name](const rclcpp::Parameter & param) {
    return param.get_name() == name;
  });
}

bool has_startup_override(
  const rclcpp::NodeOptions & options,
  const char * name)
{
  return contains_parameter(options.parameter_overrides(), name);
}
}  // namespace

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

  void warn_deprecated_arbiter_parameters(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    for (const char * name : kDeprecatedArbiterParameters) {
      const bool present = overrides.empty() ?
        has_startup_override(get_node_options(), name) :
        contains_parameter(overrides, name);
      if (present) {
        RCLCPP_WARN(
          get_logger(),
          "[%s] arbiter parameter '%s' is deprecated and ignored. Arbiter now uses STOP/FOLLOW/AVOID only.",
          kRuntimeVersion,
          name);
      }
    }
  }

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.cmd_vel_avoid_topic = get_parameter("cmd_vel_avoid_topic").as_string();
    p_.follow_command_topic = get_parameter("follow_command_topic").as_string();
    p_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    p_.publish_rate = get_parameter("publish_rate").as_double();
    p_.runtime.avoid_enter_threshold = get_parameter("avoid_enter_threshold").as_int();
    p_.runtime.avoid_exit_threshold = get_parameter("avoid_exit_threshold").as_int();
    p_.runtime.avoid_exit_hysteresis_time = get_parameter("avoid_exit_hysteresis_time").as_double();
    p_.runtime.avoid_cmd_timeout = get_parameter("avoid_cmd_timeout").as_double();
    p_.runtime.avoid_nonzero_epsilon = get_parameter("avoid_nonzero_epsilon").as_double();

    runtime_.set_config(p_.runtime);
  }

  bool needs_interface_recreation(const Params & next) const
  {
    return next.person_pose_topic != p_.person_pose_topic ||
           next.cmd_vel_follow_topic != p_.cmd_vel_follow_topic ||
           next.cmd_vel_avoid_topic != p_.cmd_vel_avoid_topic ||
           next.follow_command_topic != p_.follow_command_topic ||
           next.cmd_vel_topic != p_.cmd_vel_topic ||
           next.publish_rate != p_.publish_rate;
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = begin_recreate_lifecycle_publisher(
      cmd_pub_, preserve_activation, [this]() { publish_zero(); });

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

    restore_lifecycle_publisher(cmd_pub_, was_active);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    warn_deprecated_arbiter_parameters();
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

    candidate.publish_rate = clamp_rate_hz(candidate.publish_rate);
    candidate.runtime.avoid_enter_threshold = clamp_int_min(candidate.runtime.avoid_enter_threshold, 1);
    candidate.runtime.avoid_exit_threshold = clamp_int_min(candidate.runtime.avoid_exit_threshold, 1);
    candidate.runtime.avoid_exit_hysteresis_time = clamp_non_negative(
      candidate.runtime.avoid_exit_hysteresis_time);
    candidate.runtime.avoid_cmd_timeout = clamp_non_negative(candidate.runtime.avoid_cmd_timeout);
    candidate.runtime.avoid_nonzero_epsilon = clamp_non_negative(candidate.runtime.avoid_nonzero_epsilon);

    const bool recreate = needs_interface_recreation(candidate);
    p_ = candidate;
    runtime_.set_config(p_.runtime);
    warn_deprecated_arbiter_parameters(params);
    if (recreate) {
      recreate_interfaces(is_primary_active(*this));
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] arbiter parameters hot-reloaded.",
      kRuntimeVersion);

    return make_ok_result();
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

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Arbiter running";
    if (snapshot.stop_latched) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Arbiter stop latched";
    } else if (snapshot.mode == ArbiterMode::AVOID) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Arbiter running obstacle avoidance";
    } else if (snapshot.mode == ArbiterMode::FOLLOW) {
      message = "Arbiter forwarding follow command";
    }
    stat.summary(level, message);
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::ArbiterNode>(argc, argv);
}
