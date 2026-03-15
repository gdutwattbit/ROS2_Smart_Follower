#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/range.hpp>

#ifdef HAVE_LIBGPIOD
#include <gpiod.h>
#endif

namespace smart_follower_control
{
namespace fs = std::filesystem;

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
#ifdef HAVE_LIBGPIOD
    unsigned int trig_offset{0};
    unsigned int echo_offset{0};
    gpiod_line_request * trig_request{nullptr};
    gpiod_line_request * echo_request{nullptr};
#endif
  } left_, right_;

  double rate_{10.0};
  int window_size_{5};
  double min_range_{0.03};
  double max_range_{3.0};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};

  bool gpio_ok_{false};
  std::string gpio_backend_{"dry"};
#ifdef HAVE_LIBGPIOD
  std::string gpio_chip_path_;
#endif

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

#ifdef HAVE_LIBGPIOD
    gpio_ok_ = setup_gpiod_backend();
    gpio_backend_ = gpio_ok_ ? "libgpiod" : "dry";
#else
    gpio_ok_ = false;
    gpio_backend_ = "dry";
#endif

    if (!gpio_ok_) {
      RCLCPP_WARN(
        get_logger(),
        "No usable GPIO backend available, ultrasonic node will publish inf ranges.");
    }

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
#ifdef HAVE_LIBGPIOD
    release_sensor(left_);
    release_sensor(right_);
    gpio_chip_path_.clear();
#endif
    gpio_ok_ = false;
    gpio_backend_ = "dry";
    return CallbackReturn::SUCCESS;
  }

#ifdef HAVE_LIBGPIOD
  static gpiod_line_request * request_output_line(
    const std::string & chip_path,
    unsigned int offset,
    const char * consumer)
  {
    gpiod_chip * chip = gpiod_chip_open(chip_path.c_str());
    if (!chip) {
      return nullptr;
    }

    gpiod_line_settings * settings = gpiod_line_settings_new();
    gpiod_line_config * line_cfg = gpiod_line_config_new();
    gpiod_request_config * req_cfg = gpiod_request_config_new();
    gpiod_line_request * request = nullptr;

    if (!settings || !line_cfg || !req_cfg) {
      goto cleanup;
    }

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT) < 0) {
      goto cleanup;
    }
    if (gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE) < 0) {
      goto cleanup;
    }
    if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0) {
      goto cleanup;
    }
    gpiod_request_config_set_consumer(req_cfg, consumer);
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

cleanup:
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);
    return request;
  }

  static gpiod_line_request * request_echo_line(
    const std::string & chip_path,
    unsigned int offset,
    const char * consumer)
  {
    gpiod_chip * chip = gpiod_chip_open(chip_path.c_str());
    if (!chip) {
      return nullptr;
    }

    gpiod_line_settings * settings = gpiod_line_settings_new();
    gpiod_line_config * line_cfg = gpiod_line_config_new();
    gpiod_request_config * req_cfg = gpiod_request_config_new();
    gpiod_line_request * request = nullptr;

    if (!settings || !line_cfg || !req_cfg) {
      goto cleanup;
    }

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) < 0) {
      goto cleanup;
    }
    if (gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH) < 0) {
      goto cleanup;
    }
    if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0) {
      goto cleanup;
    }
    gpiod_request_config_set_consumer(req_cfg, consumer);
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

cleanup:
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);
    return request;
  }

  std::string detect_gpio_chip_path() const
  {
    std::error_code ec;
    for (const auto & entry : fs::directory_iterator("/dev", ec)) {
      if (ec) {
        break;
      }
      const auto name = entry.path().filename().string();
      if (name.rfind("gpiochip", 0) != 0) {
        continue;
      }

      gpiod_chip * chip = gpiod_chip_open(entry.path().c_str());
      if (!chip) {
        continue;
      }

      gpiod_chip_info * info = gpiod_chip_get_info(chip);
      std::string label;
      if (info && gpiod_chip_info_get_label(info)) {
        label = gpiod_chip_info_get_label(info);
      }
      if (info) {
        gpiod_chip_info_free(info);
      }
      gpiod_chip_close(chip);

      if (label.find("pinctrl-rp1") != std::string::npos) {
        return entry.path().string();
      }
    }

    return "/dev/gpiochip4";
  }

  bool setup_sensor(SensorConfig & cfg, const std::string & name)
  {
    cfg.trig_offset = static_cast<unsigned int>(cfg.trig_pin);
    cfg.echo_offset = static_cast<unsigned int>(cfg.echo_pin);
    cfg.trig_request = request_output_line(
      gpio_chip_path_, cfg.trig_offset, ("smart_follower_" + name + "_trig").c_str());
    if (!cfg.trig_request) {
      return false;
    }
    cfg.echo_request = request_echo_line(
      gpio_chip_path_, cfg.echo_offset, ("smart_follower_" + name + "_echo").c_str());
    if (!cfg.echo_request) {
      release_sensor(cfg);
      return false;
    }
    drain_edge_events(cfg);
    return true;
  }

  void release_sensor(SensorConfig & cfg)
  {
    if (cfg.trig_request) {
      gpiod_line_request_release(cfg.trig_request);
      cfg.trig_request = nullptr;
    }
    if (cfg.echo_request) {
      gpiod_line_request_release(cfg.echo_request);
      cfg.echo_request = nullptr;
    }
  }

  bool setup_gpiod_backend()
  {
    gpio_chip_path_ = detect_gpio_chip_path();
    if (gpio_chip_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to detect GPIO chip for libgpiod");
      return false;
    }

    if (!setup_sensor(left_, "left")) {
      RCLCPP_ERROR(get_logger(), "Failed to request left ultrasonic GPIO lines via libgpiod");
      release_sensor(left_);
      release_sensor(right_);
      return false;
    }
    if (!setup_sensor(right_, "right")) {
      RCLCPP_ERROR(get_logger(), "Failed to request right ultrasonic GPIO lines via libgpiod");
      release_sensor(left_);
      release_sensor(right_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "libgpiod backend ready on %s", gpio_chip_path_.c_str());
    return true;
  }

  void drain_edge_events(SensorConfig & cfg)
  {
    if (!cfg.echo_request) {
      return;
    }
    auto * buffer = gpiod_edge_event_buffer_new(8);
    if (!buffer) {
      return;
    }
    while (gpiod_line_request_wait_edge_events(cfg.echo_request, 0) > 0) {
      const int ret = gpiod_line_request_read_edge_events(cfg.echo_request, buffer, 8);
      if (ret <= 0) {
        break;
      }
    }
    gpiod_edge_event_buffer_free(buffer);
  }

  bool wait_for_event(
    SensorConfig & cfg,
    gpiod_edge_event_type wanted,
    int64_t timeout_ns,
    uint64_t & timestamp_ns)
  {
    auto * buffer = gpiod_edge_event_buffer_new(4);
    if (!buffer) {
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(timeout_ns);
    bool found = false;

    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      const int ready = gpiod_line_request_wait_edge_events(cfg.echo_request, remaining);
      if (ready <= 0) {
        break;
      }

      const int count = gpiod_line_request_read_edge_events(cfg.echo_request, buffer, 4);
      if (count <= 0) {
        break;
      }

      for (int i = 0; i < count; ++i) {
        gpiod_edge_event * event = gpiod_edge_event_buffer_get_event(buffer, static_cast<unsigned long>(i));
        if (!event) {
          continue;
        }
        if (gpiod_edge_event_get_line_offset(event) != cfg.echo_offset) {
          continue;
        }
        if (gpiod_edge_event_get_event_type(event) != wanted) {
          continue;
        }
        timestamp_ns = gpiod_edge_event_get_timestamp_ns(event);
        found = true;
        break;
      }
      if (found) {
        break;
      }
    }

    gpiod_edge_event_buffer_free(buffer);
    return found;
  }
#endif

  double read_distance_m(SensorConfig & cfg)
  {
#ifdef HAVE_LIBGPIOD
    if (!gpio_ok_ || !cfg.trig_request || !cfg.echo_request) {
      return std::numeric_limits<double>::infinity();
    }

    constexpr int64_t kTimeoutNs = 30000000LL;
    drain_edge_events(cfg);

    if (gpiod_line_request_set_value(
        cfg.trig_request, cfg.trig_offset, GPIOD_LINE_VALUE_INACTIVE) < 0)
    {
      return std::numeric_limits<double>::infinity();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(2));
    if (gpiod_line_request_set_value(
        cfg.trig_request, cfg.trig_offset, GPIOD_LINE_VALUE_ACTIVE) < 0)
    {
      return std::numeric_limits<double>::infinity();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    if (gpiod_line_request_set_value(
        cfg.trig_request, cfg.trig_offset, GPIOD_LINE_VALUE_INACTIVE) < 0)
    {
      return std::numeric_limits<double>::infinity();
    }

    uint64_t rise_ns = 0;
    uint64_t fall_ns = 0;
    if (!wait_for_event(cfg, GPIOD_EDGE_EVENT_RISING_EDGE, kTimeoutNs, rise_ns)) {
      return std::numeric_limits<double>::infinity();
    }
    if (!wait_for_event(cfg, GPIOD_EDGE_EVENT_FALLING_EDGE, kTimeoutNs, fall_ns)) {
      return std::numeric_limits<double>::infinity();
    }
    if (fall_ns <= rise_ns) {
      return std::numeric_limits<double>::infinity();
    }

    const double pulse_us = static_cast<double>(fall_ns - rise_ns) / 1000.0;
    return pulse_us * 0.0001715;
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
    stat.add("gpio_ok", gpio_ok_);
    stat.add("gpio_backend", gpio_backend_);
#ifdef HAVE_LIBGPIOD
    stat.add("gpio_chip_path", gpio_chip_path_);
#endif
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
