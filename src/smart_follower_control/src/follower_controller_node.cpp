#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_control/pid_controller.hpp"

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
    double control_rate{20.0};
    double target_distance{1.0};
    double theta_deadzone{0.03};
    double target_timeout{0.3};

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
  } p_;

  struct TargetState
  {
    double x{0.0};
    double y{0.0};
    double vx{0.0};
    double vy{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  } last_target_;

  PID pid_r_;
  PID pid_t_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};

  geometry_msgs::msg::Twist last_cmd_;

  void load_parameters()
  {
    p_.person_pose_topic = get_parameter("person_pose_topic").as_string();
    p_.cmd_vel_follow_topic = get_parameter("cmd_vel_follow_topic").as_string();
    p_.control_rate = get_parameter("control_rate").as_double();

    p_.target_distance = get_parameter("target_distance").as_double();
    p_.theta_deadzone = get_parameter("theta_deadzone").as_double();
    p_.target_timeout = get_parameter("target_timeout").as_double();

    p_.kp_r = get_parameter("pid_r.kp").as_double();
    p_.ki_r = get_parameter("pid_r.ki").as_double();
    p_.kd_r = get_parameter("pid_r.kd").as_double();
    p_.kp_t = get_parameter("pid_t.kp").as_double();
    p_.ki_t = get_parameter("pid_t.ki").as_double();
    p_.kd_t = get_parameter("pid_t.kd").as_double();
    p_.i_limit = get_parameter("pid_i_limit").as_double();
    p_.kaw = get_parameter("pid_kaw").as_double();

    p_.v_max = get_parameter("limits.v_max").as_double();
    p_.w_max = get_parameter("limits.w_max").as_double();
    p_.dv_max = get_parameter("limits.dv_max").as_double();
    p_.dw_max = get_parameter("limits.dw_max").as_double();

    configure_controllers();
  }

  void configure_controllers()
  {
    pid_r_.configure(p_.kp_r, p_.ki_r, p_.kd_r, p_.i_limit, p_.kaw);
    pid_t_.configure(p_.kp_t, p_.ki_t, p_.kd_t, p_.i_limit, p_.kaw);
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
    cmd_pub_.reset();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_follow_topic, 10);
    pose_sub_ = create_subscription<smart_follower_msgs::msg::PersonPoseArray>(
      p_.person_pose_topic,
      10,
      std::bind(&FollowerControllerNode::on_pose, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, p_.control_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FollowerControllerNode::on_timer, this));

    if (was_active) {
      cmd_pub_->on_activate();
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
    cmd_pub_->on_activate();
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
    cmd_pub_.reset();
    param_callback_handle_.reset();
    return CallbackReturn::SUCCESS;
  }

  void apply_parameter_override(Params & target, const rclcpp::Parameter & param)
  {
    const auto & name = param.get_name();
    if (name == "person_pose_topic") target.person_pose_topic = param.as_string();
    else if (name == "cmd_vel_follow_topic") target.cmd_vel_follow_topic = param.as_string();
    else if (name == "control_rate") target.control_rate = param.as_double();
    else if (name == "target_distance") target.target_distance = param.as_double();
    else if (name == "theta_deadzone") target.theta_deadzone = param.as_double();
    else if (name == "target_timeout") target.target_timeout = param.as_double();
    else if (name == "pid_r.kp") target.kp_r = param.as_double();
    else if (name == "pid_r.ki") target.ki_r = param.as_double();
    else if (name == "pid_r.kd") target.kd_r = param.as_double();
    else if (name == "pid_t.kp") target.kp_t = param.as_double();
    else if (name == "pid_t.ki") target.ki_t = param.as_double();
    else if (name == "pid_t.kd") target.kd_t = param.as_double();
    else if (name == "pid_i_limit") target.i_limit = param.as_double();
    else if (name == "pid_kaw") target.kaw = param.as_double();
    else if (name == "limits.v_max") target.v_max = param.as_double();
    else if (name == "limits.w_max") target.w_max = param.as_double();
    else if (name == "limits.dv_max") target.dv_max = param.as_double();
    else if (name == "limits.dw_max") target.dw_max = param.as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    candidate.control_rate = std::max(1.0, candidate.control_rate);
    candidate.target_timeout = std::max(0.0, candidate.target_timeout);
    candidate.theta_deadzone = std::max(0.0, candidate.theta_deadzone);
    candidate.i_limit = std::max(0.0, candidate.i_limit);
    candidate.v_max = std::max(0.0, candidate.v_max);
    candidate.w_max = std::max(0.0, candidate.w_max);
    candidate.dv_max = std::max(0.0, candidate.dv_max);
    candidate.dw_max = std::max(0.0, candidate.dw_max);

    p_ = candidate;
    configure_controllers();
    recreate_interfaces(this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    RCLCPP_INFO(
      get_logger(),
      "[alpha-0.0.4] controller parameters hot-reloaded: pose=%s cmd=%s rate=%.2f timeout=%.2f",
      p_.person_pose_topic.c_str(),
      p_.cmd_vel_follow_topic.c_str(),
      p_.control_rate,
      p_.target_timeout);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void on_pose(const smart_follower_msgs::msg::PersonPoseArray::SharedPtr msg)
  {
    if (!msg || msg->lock_id < 0) {
      last_target_.valid = false;
      return;
    }

    bool updated = false;
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

      last_target_.x = p.position.x;
      last_target_.y = p.position.y;
      last_target_.vx = 0.0;
      last_target_.vy = 0.0;
      last_target_.stamp = rclcpp::Time(msg->header.stamp);
      last_target_.valid = true;
      updated = true;
      break;
    }

    if (!updated) {
      last_target_.valid = false;
    }
  }

  std::optional<TargetState> predict_target(const rclcpp::Time & now)
  {
    if (!last_target_.valid) {
      return std::nullopt;
    }

    const double age = (now - last_target_.stamp).seconds();
    if (age < 0.0 || age > p_.target_timeout) {
      last_target_.valid = false;
      return std::nullopt;
    }

    return last_target_;
  }

  void on_timer()
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    const auto now = this->now();
    auto target_opt = predict_target(now);
    if (!target_opt.has_value()) {
      publish_zero();
      return;
    }

    const auto & t = target_opt.value();
    const double rho = std::sqrt(t.x * t.x + t.y * t.y);
    double theta = std::atan2(t.y, t.x);
    if (std::abs(theta) < p_.theta_deadzone) {
      theta = 0.0;
    }

    const double dt = 1.0 / std::max(1.0, p_.control_rate);
    const double e_r = rho - p_.target_distance;
    const double e_t = theta;

    double v = pid_r_.update(e_r, dt, -p_.v_max, p_.v_max);
    double w = pid_t_.update(e_t, dt, -p_.w_max, p_.w_max);

    if (std::abs(theta) > 0.5) {
      v *= 0.3;
    }

    v = rate_limit(v, last_cmd_.linear.x, p_.dv_max, dt);
    w = rate_limit(w, last_cmd_.angular.z, p_.dw_max, dt);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    last_cmd_ = cmd;

    if (cmd_pub_ && cmd_pub_->is_activated()) {
      cmd_pub_->publish(cmd);
    }

    diagnostics_.force_update();
  }

  double rate_limit(double target, double current, double accel_limit, double dt) const
  {
    const double delta_max = std::max(0.0, accel_limit) * std::max(1e-3, dt);
    return std::clamp(target, current - delta_max, current + delta_max);
  }

  void publish_zero()
  {
    pid_r_.reset();
    pid_t_.reset();
    geometry_msgs::msg::Twist cmd;
    last_cmd_ = cmd;
    if (cmd_pub_ && cmd_pub_->is_activated()) {
      cmd_pub_->publish(cmd);
    }
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.add("last_cmd_v", last_cmd_.linear.x);
    stat.add("last_cmd_w", last_cmd_.angular.z);
    stat.add("target_valid", last_target_.valid);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Follower control active");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_control::FollowerControllerNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
