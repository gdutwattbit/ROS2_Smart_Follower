#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/arbiter_state_machine.hpp"

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
    ArbiterThresholds thresholds;
    double degraded_linear_scale{0.5};
    double search_angular_speed{0.3};

    int avoid_enter_threshold{3};
    int avoid_exit_threshold{5};
    double avoid_exit_hysteresis_time{0.2};
    double avoid_cmd_timeout{0.2};
    double avoid_nonzero_epsilon{1e-3};
  } p_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follow_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr avoid_sub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_{this};

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

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.cmd_vel_avoid_topic = get_parameter("cmd_vel_avoid_topic").as_string();
    p_.follow_command_topic = get_parameter("follow_command_topic").as_string();
    p_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();

    p_.publish_rate = get_parameter("publish_rate").as_double();
    p_.thresholds.lost_time_normal_max = get_parameter("lost_time_normal_max").as_double();
    p_.thresholds.lost_time_degraded_max = get_parameter("lost_time_degraded_max").as_double();
    p_.thresholds.lost_time_search_max = get_parameter("lost_time_search_max").as_double();
    p_.degraded_linear_scale = get_parameter("degraded_linear_scale").as_double();
    p_.search_angular_speed = get_parameter("search_angular_speed").as_double();

    p_.avoid_enter_threshold = get_parameter("avoid_enter_threshold").as_int();
    p_.avoid_exit_threshold = get_parameter("avoid_exit_threshold").as_int();
    p_.avoid_exit_hysteresis_time = get_parameter("avoid_exit_hysteresis_time").as_double();
    p_.avoid_cmd_timeout = get_parameter("avoid_cmd_timeout").as_double();
    p_.avoid_nonzero_epsilon = get_parameter("avoid_nonzero_epsilon").as_double();
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation && cmd_pub_ && cmd_pub_->is_activated();
    if (was_active) {
      publish_zero();
      cmd_pub_->on_deactivate();
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
      std::bind(&ArbiterNode::on_person_pose, this, std::placeholders::_1));
    follow_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_follow_topic,
      10,
      std::bind(&ArbiterNode::on_follow_cmd, this, std::placeholders::_1));
    avoid_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_avoid_topic,
      10,
      std::bind(&ArbiterNode::on_avoid_cmd, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<smart_follower_msgs::msg::FollowCommand>(
      p_.follow_command_topic,
      10,
      std::bind(&ArbiterNode::on_user_cmd, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, p_.publish_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ArbiterNode::on_timer, this));

    if (was_active) {
      cmd_pub_->on_activate();
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
    cmd_pub_->on_activate();
    mode_ = ArbiterMode::STOP;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    publish_zero();
    if (cmd_pub_) {
      cmd_pub_->on_deactivate();
    }
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
    else if (name == "lost_time_normal_max") target.thresholds.lost_time_normal_max = param.as_double();
    else if (name == "lost_time_degraded_max") target.thresholds.lost_time_degraded_max = param.as_double();
    else if (name == "lost_time_search_max") target.thresholds.lost_time_search_max = param.as_double();
    else if (name == "degraded_linear_scale") target.degraded_linear_scale = param.as_double();
    else if (name == "search_angular_speed") target.search_angular_speed = param.as_double();
    else if (name == "avoid_enter_threshold") target.avoid_enter_threshold = param.as_int();
    else if (name == "avoid_exit_threshold") target.avoid_exit_threshold = param.as_int();
    else if (name == "avoid_exit_hysteresis_time") target.avoid_exit_hysteresis_time = param.as_double();
    else if (name == "avoid_cmd_timeout") target.avoid_cmd_timeout = param.as_double();
    else if (name == "avoid_nonzero_epsilon") target.avoid_nonzero_epsilon = param.as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    candidate.publish_rate = std::max(1.0, candidate.publish_rate);
    candidate.thresholds.lost_time_normal_max = std::max(0.0, candidate.thresholds.lost_time_normal_max);
    candidate.thresholds.lost_time_degraded_max = std::max(candidate.thresholds.lost_time_normal_max, candidate.thresholds.lost_time_degraded_max);
    candidate.thresholds.lost_time_search_max = std::max(candidate.thresholds.lost_time_degraded_max, candidate.thresholds.lost_time_search_max);
    candidate.degraded_linear_scale = std::clamp(candidate.degraded_linear_scale, 0.0, 1.0);
    candidate.avoid_enter_threshold = std::max(1, candidate.avoid_enter_threshold);
    candidate.avoid_exit_threshold = std::max(1, candidate.avoid_exit_threshold);
    candidate.avoid_exit_hysteresis_time = std::max(0.0, candidate.avoid_exit_hysteresis_time);
    candidate.avoid_cmd_timeout = std::max(0.0, candidate.avoid_cmd_timeout);
    candidate.avoid_nonzero_epsilon = std::max(0.0, candidate.avoid_nonzero_epsilon);

    p_ = candidate;
    recreate_interfaces(this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    RCLCPP_INFO(
      get_logger(),
      "[alpha-0.0.4] arbiter parameters hot-reloaded: pose=%s follow=%s avoid=%s cmd=%s rate=%.2f",
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

  void on_person_pose(const smart_follower_msgs::msg::PersonPoseArray::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (msg->lock_id < 0 || msg->lock_state != smart_follower_msgs::msg::PersonPoseArray::LOCKED) {
      return;
    }

    for (const auto & p : msg->persons) {
      if (p.track_id != msg->lock_id) {
        continue;
      }
      if (p.track_state != smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
        continue;
      }
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y)) {
        continue;
      }
      last_target_time_ = rclcpp::Time(msg->header.stamp);
      last_target_theta_ = std::atan2(p.position.y, p.position.x);
      break;
    }
  }

  void on_follow_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (msg) {
      latest_follow_cmd_ = *msg;
    }
  }

  void on_avoid_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_avoid_cmd_ = *msg;
    last_avoid_time_ = now();

    const bool nonzero = is_nonzero(latest_avoid_cmd_);
    if (nonzero) {
      avoid_nonzero_count_++;
      avoid_zero_count_ = 0;
      if (avoid_nonzero_count_ >= p_.avoid_enter_threshold) {
        avoid_latched_ = true;
      }
    } else {
      avoid_zero_count_++;
      avoid_nonzero_count_ = 0;
      if (avoid_zero_count_ == 1) {
        avoid_zero_start_ = now();
      }
    }
  }

  void on_user_cmd(const smart_follower_msgs::msg::FollowCommand::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (msg->command == smart_follower_msgs::msg::FollowCommand::ESTOP) {
      stop_latched_ = true;
      mode_ = ArbiterMode::STOP;
      latest_follow_cmd_ = geometry_msgs::msg::Twist();
      latest_avoid_cmd_ = geometry_msgs::msg::Twist();
    } else if (msg->command == smart_follower_msgs::msg::FollowCommand::RESET) {
      stop_latched_ = false;
      mode_ = ArbiterMode::STOP;
      avoid_nonzero_count_ = 0;
      avoid_zero_count_ = 0;
      avoid_latched_ = false;
      latest_follow_cmd_ = geometry_msgs::msg::Twist();
      latest_avoid_cmd_ = geometry_msgs::msg::Twist();
    }
  }

  bool is_nonzero(const geometry_msgs::msg::Twist & cmd) const
  {
    return std::abs(cmd.linear.x) > p_.avoid_nonzero_epsilon ||
           std::abs(cmd.angular.z) > p_.avoid_nonzero_epsilon;
  }

  bool avoid_valid(const rclcpp::Time & now_time)
  {
    if ((now_time - last_avoid_time_).seconds() > p_.avoid_cmd_timeout) {
      latest_avoid_cmd_ = geometry_msgs::msg::Twist();
      avoid_latched_ = false;
      return false;
    }

    if (!avoid_latched_) {
      return false;
    }

    if (!is_nonzero(latest_avoid_cmd_) && avoid_zero_count_ >= p_.avoid_exit_threshold &&
      (now_time - avoid_zero_start_).seconds() >= p_.avoid_exit_hysteresis_time)
    {
      avoid_latched_ = false;
      return false;
    }

    return avoid_latched_;
  }

  void on_timer()
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    const auto now_time = now();
    const bool avoid_on = avoid_valid(now_time);

    double age = std::numeric_limits<double>::infinity();
    if (last_target_time_.nanoseconds() > 0) {
      age = (now_time - last_target_time_).seconds();
    }

    if (!stop_latched_ && age > p_.thresholds.lost_time_search_max) {
      stop_latched_ = true;
    }

    if (stop_latched_) {
      mode_ = ArbiterMode::STOP;
    } else if (avoid_on) {
      mode_ = ArbiterMode::AVOID;
    } else {
      mode_ = select_follow_mode(age, p_.thresholds);
    }

    geometry_msgs::msg::Twist out;
    switch (mode_) {
      case ArbiterMode::FOLLOW_NORMAL:
        out = latest_follow_cmd_;
        break;
      case ArbiterMode::FOLLOW_DEGRADED:
        out = latest_follow_cmd_;
        out.linear.x *= p_.degraded_linear_scale;
        if (!std::isfinite(out.angular.z) || std::abs(out.angular.z) < 1e-6) {
          out.angular.z = std::clamp(last_target_theta_, -0.8, 0.8);
        }
        break;
      case ArbiterMode::SEARCH:
        out.linear.x = 0.0;
        out.angular.z = p_.search_angular_speed;
        break;
      case ArbiterMode::AVOID:
        out = latest_avoid_cmd_;
        break;
      case ArbiterMode::STOP:
      default:
        out = geometry_msgs::msg::Twist();
        break;
    }

    if (cmd_pub_ && cmd_pub_->is_activated()) {
      cmd_pub_->publish(out);
    }

    diagnostics_.force_update();
  }

  void publish_zero()
  {
    if (cmd_pub_ && cmd_pub_->is_activated()) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.add("mode", static_cast<int>(mode_));
    stat.add("stop_latched", stop_latched_);
    stat.add("avoid_latched", avoid_latched_);
    stat.add("last_target_age_s", last_target_time_.nanoseconds() > 0 ? (now() - last_target_time_).seconds() : -1.0);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Arbiter running");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_control::ArbiterNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
