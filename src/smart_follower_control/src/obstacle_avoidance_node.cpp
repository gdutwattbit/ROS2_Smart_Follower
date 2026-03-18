#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"
#include "smart_follower_control/obstacle_runtime.hpp"

namespace smart_follower_control
{

class ObstacleAvoidanceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ObstacleAvoidanceNode()
  : rclcpp_lifecycle::LifecycleNode("obstacle_avoidance_node")
  {
    declare_parameter("left_range_topic", std::string("left_ultrasonic/range"));
    declare_parameter("right_range_topic", std::string("right_ultrasonic/range"));
    declare_parameter("depth_topic", std::string("/camera/depth/image_raw"));
    declare_parameter("cmd_vel_input_topic", std::string("/cmd_vel"));
    declare_parameter("cmd_vel_avoid_topic", std::string("cmd_vel_avoid"));
    declare_parameter("rate", 20.0);
    declare_parameter("d_min", 0.12);
    declare_parameter("t_react", 0.20);
    declare_parameter("a_brake", 0.8);
    declare_parameter("margin", 0.08);
    declare_parameter("exit_margin", 0.08);
    declare_parameter("depth_roi_width_ratio", 0.4);
    declare_parameter("depth_roi_height_ratio", 0.35);
    declare_parameter("depth_percentile", 0.05);
    declare_parameter("depth_sample_stride", 2);
    declare_parameter("turn_speed", 0.5);
    declare_parameter("slow_turn_speed", 0.25);
    declare_parameter("back_speed", -0.15);
  }

private:
  struct Params
  {
    std::string left_topic{"left_ultrasonic/range"};
    std::string right_topic{"right_ultrasonic/range"};
    std::string depth_topic{"/camera/depth/image_raw"};
    std::string cmd_vel_input_topic{"/cmd_vel"};
    std::string cmd_vel_avoid_topic{"cmd_vel_avoid"};
    double rate{20.0};
    ObstacleRuntimeConfig runtime;
  } p_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr avoid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};
  ObstacleRuntime runtime_;

  void load_parameters()
  {
    p_.left_topic = get_parameter("left_range_topic").as_string();
    p_.right_topic = get_parameter("right_range_topic").as_string();
    p_.depth_topic = get_parameter("depth_topic").as_string();
    p_.cmd_vel_input_topic = get_parameter("cmd_vel_input_topic").as_string();
    p_.cmd_vel_avoid_topic = get_parameter("cmd_vel_avoid_topic").as_string();
    p_.rate = get_parameter("rate").as_double();
    p_.runtime.d_min = get_parameter("d_min").as_double();
    p_.runtime.t_react = get_parameter("t_react").as_double();
    p_.runtime.a_brake = get_parameter("a_brake").as_double();
    p_.runtime.margin = get_parameter("margin").as_double();
    p_.runtime.exit_margin = get_parameter("exit_margin").as_double();
    p_.runtime.depth_roi_width_ratio = get_parameter("depth_roi_width_ratio").as_double();
    p_.runtime.depth_roi_height_ratio = get_parameter("depth_roi_height_ratio").as_double();
    p_.runtime.depth_percentile = get_parameter("depth_percentile").as_double();
    p_.runtime.depth_sample_stride = std::max<int>(1, static_cast<int>(get_parameter("depth_sample_stride").as_int()));
    p_.runtime.turn_speed = get_parameter("turn_speed").as_double();
    p_.runtime.slow_turn_speed = get_parameter("slow_turn_speed").as_double();
    p_.runtime.back_speed = get_parameter("back_speed").as_double();
    runtime_.set_config(p_.runtime);
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation && publisher_is_activated(avoid_pub_);
    if (was_active) {
      publish_zero();
      deactivate_publisher(avoid_pub_);
    }

    timer_.reset();
    left_sub_.reset();
    right_sub_.reset();
    depth_sub_.reset();
    vel_sub_.reset();
    avoid_pub_.reset();
    runtime_.clear();

    left_sub_ = create_subscription<sensor_msgs::msg::Range>(
      p_.left_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        if (msg) {
          runtime_.on_left_range(msg->range, rclcpp::Time(msg->header.stamp));
        }
      });
    right_sub_ = create_subscription<sensor_msgs::msg::Range>(
      p_.right_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        if (msg) {
          runtime_.on_right_range(msg->range, rclcpp::Time(msg->header.stamp));
        }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      p_.depth_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { runtime_.on_depth(msg); });
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_input_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          runtime_.on_cmd_vel(*msg);
        }
      });

    avoid_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_avoid_topic, 10);
    timer_ = create_wall_timer(hz_to_period(p_.rate), std::bind(&ObstacleAvoidanceNode::on_timer, this));

    if (was_active) {
      activate_publisher(avoid_pub_);
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    recreate_interfaces(false);

    diagnostics_.setHardwareID("smart_follower_avoidance");
    diagnostics_.add("avoidance_status", this, &ObstacleAvoidanceNode::diag_callback);
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ObstacleAvoidanceNode::on_parameters_set, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    activate_publisher(avoid_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    publish_zero();
    deactivate_publisher(avoid_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    left_sub_.reset();
    right_sub_.reset();
    depth_sub_.reset();
    vel_sub_.reset();
    avoid_pub_.reset();
    param_callback_handle_.reset();
    runtime_.clear();
    return CallbackReturn::SUCCESS;
  }

  void apply_parameter_override(Params & target, const rclcpp::Parameter & param)
  {
    const auto & name = param.get_name();
    if (name == "left_range_topic") target.left_topic = param.as_string();
    else if (name == "right_range_topic") target.right_topic = param.as_string();
    else if (name == "depth_topic") target.depth_topic = param.as_string();
    else if (name == "cmd_vel_input_topic") target.cmd_vel_input_topic = param.as_string();
    else if (name == "cmd_vel_avoid_topic") target.cmd_vel_avoid_topic = param.as_string();
    else if (name == "rate") target.rate = param.as_double();
    else if (name == "d_min") target.runtime.d_min = param.as_double();
    else if (name == "t_react") target.runtime.t_react = param.as_double();
    else if (name == "a_brake") target.runtime.a_brake = param.as_double();
    else if (name == "margin") target.runtime.margin = param.as_double();
    else if (name == "exit_margin") target.runtime.exit_margin = param.as_double();
    else if (name == "depth_roi_width_ratio") target.runtime.depth_roi_width_ratio = param.as_double();
    else if (name == "depth_roi_height_ratio") target.runtime.depth_roi_height_ratio = param.as_double();
    else if (name == "depth_percentile") target.runtime.depth_percentile = param.as_double();
    else if (name == "depth_sample_stride") target.runtime.depth_sample_stride = static_cast<int>(param.as_int());
    else if (name == "turn_speed") target.runtime.turn_speed = param.as_double();
    else if (name == "slow_turn_speed") target.runtime.slow_turn_speed = param.as_double();
    else if (name == "back_speed") target.runtime.back_speed = param.as_double();
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    candidate.rate = std::max(1.0, candidate.rate);
    candidate.runtime.a_brake = std::max(1e-3, candidate.runtime.a_brake);
    candidate.runtime.depth_roi_width_ratio = std::clamp(candidate.runtime.depth_roi_width_ratio, 0.05, 1.0);
    candidate.runtime.depth_roi_height_ratio = std::clamp(candidate.runtime.depth_roi_height_ratio, 0.05, 1.0);
    candidate.runtime.depth_percentile = std::clamp(candidate.runtime.depth_percentile, 0.0, 1.0);
    candidate.runtime.depth_sample_stride = std::max(1, candidate.runtime.depth_sample_stride);

    p_ = candidate;
    runtime_.set_config(p_.runtime);
    recreate_interfaces(is_primary_active(*this));

    RCLCPP_INFO(
      get_logger(),
      "[%s] avoidance parameters hot-reloaded: left=%s right=%s depth=%s cmd=%s rate=%.2f stride=%d",
      kRuntimeVersion,
      p_.left_topic.c_str(),
      p_.right_topic.c_str(),
      p_.depth_topic.c_str(),
      p_.cmd_vel_avoid_topic.c_str(),
      p_.rate,
      p_.runtime.depth_sample_stride);

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

    publish_if_activated(avoid_pub_, runtime_.compute_command(now()));
    diagnostics_.force_update();
  }

  void publish_zero()
  {
    runtime_.clear();
    publish_if_activated(avoid_pub_, geometry_msgs::msg::Twist());
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto snapshot = runtime_.snapshot(now());
    stat.add("left_dist", snapshot.left_dist);
    stat.add("right_dist", snapshot.right_dist);
    stat.add("depth_dist", snapshot.depth_dist);
    stat.add("left_age_s", snapshot.left_age_s);
    stat.add("right_age_s", snapshot.right_age_s);
    stat.add("depth_age_s", snapshot.depth_age_s);
    stat.add("depth_message_count", snapshot.depth_message_count);
    stat.add("depth_process_count", snapshot.depth_process_count);
    stat.add("depth_sample_stride", snapshot.depth_sample_stride);
    stat.add("current_speed", snapshot.current_speed);

    const bool left_stale = snapshot.left_age_s < 0.0 || snapshot.left_age_s > 1.0;
    const bool right_stale = snapshot.right_age_s < 0.0 || snapshot.right_age_s > 1.0;
    const bool depth_stale = snapshot.depth_age_s < 0.0 || snapshot.depth_age_s > 1.0 || snapshot.depth_process_count == 0;

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Obstacle avoidance active";
    if (left_stale && right_stale && depth_stale) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Obstacle inputs unavailable";
    } else if (left_stale || right_stale || depth_stale || snapshot.depth_message_count == 0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Obstacle inputs degraded";
    }
    stat.summary(level, message);
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::ObstacleAvoidanceNode>(argc, argv);
}
