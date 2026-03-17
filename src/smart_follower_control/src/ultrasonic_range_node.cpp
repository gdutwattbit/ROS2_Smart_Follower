#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "smart_follower_control/constants.hpp"
#include "smart_follower_control/lifecycle_utils.hpp"
#include "smart_follower_control/ultrasonic_runtime.hpp"

namespace smart_follower_control
{

class UltrasonicRangeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  UltrasonicRangeNode()
  : rclcpp_lifecycle::LifecycleNode("ultrasonic_range_node")
  {
    declare_parameter("rate", 10.0);
    declare_parameter("window_size", 5);
    declare_parameter("max_range", 3.0);
    declare_parameter("min_range", 0.03);
    declare_parameter("left.trig_pin", 23);
    declare_parameter("left.echo_pin", 24);
    declare_parameter("right.trig_pin", 4);
    declare_parameter("right.echo_pin", 14);
    declare_parameter("left.topic", std::string("left_ultrasonic/range"));
    declare_parameter("right.topic", std::string("right_ultrasonic/range"));
    declare_parameter("frame_left", std::string("ultrasonic_left_link"));
    declare_parameter("frame_right", std::string("ultrasonic_right_link"));
  }

private:
  struct Params
  {
    UltrasonicRuntimeConfig runtime;
  } p_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};
  UltrasonicRuntime runtime_;

  void load_parameters()
  {
    p_.runtime.rate = get_parameter("rate").as_double();
    p_.runtime.window_size = std::max<int>(1, static_cast<int>(get_parameter("window_size").as_int()));
    p_.runtime.max_range = get_parameter("max_range").as_double();
    p_.runtime.min_range = get_parameter("min_range").as_double();
    p_.runtime.left.trig_pin = get_parameter("left.trig_pin").as_int();
    p_.runtime.left.echo_pin = get_parameter("left.echo_pin").as_int();
    p_.runtime.right.trig_pin = get_parameter("right.trig_pin").as_int();
    p_.runtime.right.echo_pin = get_parameter("right.echo_pin").as_int();
    p_.runtime.left.topic = get_parameter("left.topic").as_string();
    p_.runtime.right.topic = get_parameter("right.topic").as_string();
    p_.runtime.left.frame = get_parameter("frame_left").as_string();
    p_.runtime.right.frame = get_parameter("frame_right").as_string();
    runtime_.set_config(p_.runtime);
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation &&
      publisher_is_activated(left_pub_) && publisher_is_activated(right_pub_);
    if (was_active) {
      deactivate_publisher(left_pub_);
      deactivate_publisher(right_pub_);
    }

    timer_.reset();
    left_pub_.reset();
    right_pub_.reset();

    left_pub_ = create_publisher<sensor_msgs::msg::Range>(p_.runtime.left.topic, 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Range>(p_.runtime.right.topic, 10);
    runtime_.reinitialize(get_logger());
    timer_ = create_wall_timer(hz_to_period(p_.runtime.rate), std::bind(&UltrasonicRangeNode::on_timer, this));

    if (was_active) {
      activate_publisher(left_pub_);
      activate_publisher(right_pub_);
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    recreate_interfaces(false);

    diagnostics_.setHardwareID("smart_follower_ultrasonic");
    diagnostics_.add("ultrasonic_status", this, &UltrasonicRangeNode::diag_callback);
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&UltrasonicRangeNode::on_parameters_set, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    activate_publisher(left_pub_);
    activate_publisher(right_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    deactivate_publisher(left_pub_);
    deactivate_publisher(right_pub_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    left_pub_.reset();
    right_pub_.reset();
    param_callback_handle_.reset();
    runtime_.clear();
    return CallbackReturn::SUCCESS;
  }

  void apply_parameter_override(Params & target, const rclcpp::Parameter & param)
  {
    const auto & name = param.get_name();
    if (name == "rate") target.runtime.rate = param.as_double();
    else if (name == "window_size") target.runtime.window_size = static_cast<int>(param.as_int());
    else if (name == "max_range") target.runtime.max_range = param.as_double();
    else if (name == "min_range") target.runtime.min_range = param.as_double();
    else if (name == "left.trig_pin") target.runtime.left.trig_pin = param.as_int();
    else if (name == "left.echo_pin") target.runtime.left.echo_pin = param.as_int();
    else if (name == "right.trig_pin") target.runtime.right.trig_pin = param.as_int();
    else if (name == "right.echo_pin") target.runtime.right.echo_pin = param.as_int();
    else if (name == "left.topic") target.runtime.left.topic = param.as_string();
    else if (name == "right.topic") target.runtime.right.topic = param.as_string();
    else if (name == "frame_left") target.runtime.left.frame = param.as_string();
    else if (name == "frame_right") target.runtime.right.frame = param.as_string();
  }

  void log_hot_reload()
  {
    RCLCPP_INFO(
      get_logger(),
      "[%s] ultrasonic parameters hot-reloaded: rate=%.2f left=(%d,%d,%s) right=(%d,%d,%s) backend=%s",
      kRuntimeVersion,
      p_.runtime.rate,
      p_.runtime.left.trig_pin,
      p_.runtime.left.echo_pin,
      p_.runtime.left.topic.c_str(),
      p_.runtime.right.trig_pin,
      p_.runtime.right.echo_pin,
      p_.runtime.right.topic.c_str(),
      runtime_.gpio_backend().c_str());
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & params)
  {
    Params candidate = p_;
    for (const auto & param : params) {
      apply_parameter_override(candidate, param);
    }

    candidate.runtime.rate = std::max(1.0, candidate.runtime.rate);
    candidate.runtime.window_size = std::max(1, candidate.runtime.window_size);
    candidate.runtime.min_range = std::max(0.0, candidate.runtime.min_range);
    candidate.runtime.max_range = std::max(candidate.runtime.min_range, candidate.runtime.max_range);

    p_ = candidate;
    runtime_.set_config(p_.runtime);

    if (is_primary_active(*this)) {
      runtime_.request_reconfigure();
    } else {
      recreate_interfaces(false);
      log_hot_reload();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void publish_range(
    const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr & pub,
    const std::string & frame,
    double range,
    const rclcpp::Time & stamp)
  {
    if (!publisher_is_activated(pub)) {
      return;
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : now();
    msg.header.frame_id = frame;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.52;
    msg.min_range = p_.runtime.min_range;
    msg.max_range = p_.runtime.max_range;
    if (std::isfinite(range)) {
      msg.range = static_cast<float>(std::clamp(range, p_.runtime.min_range, p_.runtime.max_range));
    } else {
      msg.range = std::numeric_limits<float>::infinity();
    }
    pub->publish(msg);
  }

  void on_timer()
  {
    if (!is_primary_active(*this)) {
      return;
    }

    if (runtime_.consume_reconfigure_request()) {
      recreate_interfaces(true);
      log_hot_reload();
      return;
    }

    runtime_.measure_step(now());
    publish_range(left_pub_, p_.runtime.left.frame, runtime_.last_left_range(), runtime_.last_left_stamp());
    publish_range(right_pub_, p_.runtime.right.frame, runtime_.last_right_range(), runtime_.last_right_stamp());
    diagnostics_.force_update();
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto snapshot = runtime_.snapshot(now());
    stat.add("gpio_ok", snapshot.gpio_ok);
    stat.add("gpio_backend", snapshot.gpio_backend);
    stat.add("gpio_chip_path", snapshot.gpio_chip_path);
    stat.add("last_left_range", snapshot.last_left_range);
    stat.add("last_right_range", snapshot.last_right_range);
    stat.add("last_left_age_s", snapshot.last_left_age_s);
    stat.add("last_right_age_s", snapshot.last_right_age_s);
    stat.add("measure_left_next", snapshot.measure_left_next);
    stat.add("left_samples", snapshot.left_samples);
    stat.add("right_samples", snapshot.right_samples);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Ultrasonic node active");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  return smart_follower_control::run_lifecycle_node<smart_follower_control::UltrasonicRangeNode>(argc, argv);
}
