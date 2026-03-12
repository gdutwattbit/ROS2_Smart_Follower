#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/range.hpp>

#ifdef HAVE_PIGPIO
#include <pigpio.h>
#endif

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
    declare_parameter("right.trig_pin", 27);
    declare_parameter("right.echo_pin", 22);

    declare_parameter("left.topic", std::string("left_ultrasonic/range"));
    declare_parameter("right.topic", std::string("right_ultrasonic/range"));
    declare_parameter("frame_left", std::string("ultrasonic_left_link"));
    declare_parameter("frame_right", std::string("ultrasonic_right_link"));
  }

private:
  struct SensorConfig
  {
    int trig_pin{0};
    int echo_pin{0};
    std::string topic;
    std::string frame;
    std::deque<double> history;
  } left_, right_;

  double rate_{10.0};
  int window_size_{5};
  double min_range_{0.03};
  double max_range_{3.0};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};

  bool pigpio_ok_{false};

  void load_parameters()
  {
    rate_ = get_parameter("rate").as_double();
    window_size_ = std::max<int>(1, static_cast<int>(get_parameter("window_size").as_int()));
    max_range_ = get_parameter("max_range").as_double();
    min_range_ = get_parameter("min_range").as_double();

    left_.trig_pin = get_parameter("left.trig_pin").as_int();
    left_.echo_pin = get_parameter("left.echo_pin").as_int();
    right_.trig_pin = get_parameter("right.trig_pin").as_int();
    right_.echo_pin = get_parameter("right.echo_pin").as_int();

    left_.topic = get_parameter("left.topic").as_string();
    right_.topic = get_parameter("right.topic").as_string();
    left_.frame = get_parameter("frame_left").as_string();
    right_.frame = get_parameter("frame_right").as_string();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();

    left_pub_ = create_publisher<sensor_msgs::msg::Range>(left_.topic, 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Range>(right_.topic, 10);

#ifdef HAVE_PIGPIO
    pigpio_ok_ = (gpioInitialise() >= 0);
    if (pigpio_ok_) {
      gpioSetMode(left_.trig_pin, PI_OUTPUT);
      gpioSetMode(left_.echo_pin, PI_INPUT);
      gpioSetMode(right_.trig_pin, PI_OUTPUT);
      gpioSetMode(right_.echo_pin, PI_INPUT);
      gpioWrite(left_.trig_pin, PI_LOW);
      gpioWrite(right_.trig_pin, PI_LOW);
    }
#else
    pigpio_ok_ = false;
#endif

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate_))),
      std::bind(&UltrasonicRangeNode::on_timer, this));

    diagnostics_.setHardwareID("smart_follower_ultrasonic");
    diagnostics_.add("ultrasonic_status", this, &UltrasonicRangeNode::diag_callback);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    left_pub_->on_activate();
    right_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (left_pub_) left_pub_->on_deactivate();
    if (right_pub_) right_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    left_pub_.reset();
    right_pub_.reset();
#ifdef HAVE_PIGPIO
    if (pigpio_ok_) {
      gpioTerminate();
    }
#endif
    pigpio_ok_ = false;
    return CallbackReturn::SUCCESS;
  }

  double read_distance_m(SensorConfig & cfg)
  {
#ifdef HAVE_PIGPIO
    if (!pigpio_ok_) {
      return std::numeric_limits<double>::infinity();
    }

    gpioWrite(cfg.trig_pin, PI_LOW);
    gpioDelay(2);
    gpioWrite(cfg.trig_pin, PI_HIGH);
    gpioDelay(10);
    gpioWrite(cfg.trig_pin, PI_LOW);

    uint32_t start_tick = gpioTick();
    const uint32_t timeout_us = 30000;

    while (gpioRead(cfg.echo_pin) == PI_LOW) {
      if ((gpioTick() - start_tick) > timeout_us) {
        return std::numeric_limits<double>::infinity();
      }
    }
    uint32_t echo_start = gpioTick();

    while (gpioRead(cfg.echo_pin) == PI_HIGH) {
      if ((gpioTick() - echo_start) > timeout_us) {
        return std::numeric_limits<double>::infinity();
      }
    }
    uint32_t echo_end = gpioTick();

    const double pulse_us = static_cast<double>(echo_end - echo_start);
    const double distance = pulse_us * 0.0001715;
    return distance;
#else
    (void)cfg;
    return std::numeric_limits<double>::infinity();
#endif
  }

  double median_filtered(SensorConfig & cfg, double sample)
  {
    if (!std::isfinite(sample)) {
      return std::numeric_limits<double>::infinity();
    }
    cfg.history.push_back(sample);
    while (static_cast<int>(cfg.history.size()) > window_size_) {
      cfg.history.pop_front();
    }
    std::vector<double> tmp(cfg.history.begin(), cfg.history.end());
    std::sort(tmp.begin(), tmp.end());
    return tmp[tmp.size() / 2];
  }

  void publish_range(
    const rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr & pub,
    const std::string & frame,
    double range)
  {
    if (!pub || !pub->is_activated()) {
      return;
    }

    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.52;
    msg.min_range = min_range_;
    msg.max_range = max_range_;
    if (std::isfinite(range)) {
      msg.range = static_cast<float>(std::clamp(range, min_range_, max_range_));
    } else {
      msg.range = std::numeric_limits<float>::infinity();
    }
    pub->publish(msg);
  }

  void on_timer()
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    const double left = median_filtered(left_, read_distance_m(left_));
    const double right = median_filtered(right_, read_distance_m(right_));

    publish_range(left_pub_, left_.frame, left);
    publish_range(right_pub_, right_.frame, right);

    diagnostics_.force_update();
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.add("pigpio_ok", pigpio_ok_);
    stat.add("left_samples", static_cast<int>(left_.history.size()));
    stat.add("right_samples", static_cast<int>(right_.history.size()));
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Ultrasonic node active");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_control::UltrasonicRangeNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}




