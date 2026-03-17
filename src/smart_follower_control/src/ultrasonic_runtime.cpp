#include "smart_follower_control/ultrasonic_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#ifdef HAVE_LIBGPIOD
#include <gpiod.h>
#endif

namespace smart_follower_control
{
namespace fs = std::filesystem;

struct UltrasonicRuntime::Impl
{
  struct SensorState
  {
    UltrasonicSensorConfig config;
    std::deque<double> history;
#ifdef HAVE_LIBGPIOD
    unsigned int trig_offset{0};
    unsigned int echo_offset{0};
    gpiod_line_request * trig_request{nullptr};
    gpiod_line_request * echo_request{nullptr};
#endif
  };

  UltrasonicRuntimeConfig config_;
  SensorState left_;
  SensorState right_;

  bool reconfigure_pending_{false};
  bool gpio_ok_{false};
  std::string gpio_backend_{"dry"};
  std::string gpio_chip_path_;
  double last_left_range_{std::numeric_limits<double>::infinity()};
  double last_right_range_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_left_measure_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_measure_stamp_{0, 0, RCL_ROS_TIME};
  bool measure_left_next_{true};

  void sync_sensor_configs()
  {
    left_.config = config_.left;
    right_.config = config_.right;
  }

#ifdef HAVE_LIBGPIOD
  static gpiod_line_request * request_output_line(const std::string & chip_path, unsigned int offset, const char * consumer)
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

  static gpiod_line_request * request_echo_line(const std::string & chip_path, unsigned int offset, const char * consumer)
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

  void release_sensor(SensorState & state)
  {
    if (state.trig_request) {
      gpiod_line_request_release(state.trig_request);
      state.trig_request = nullptr;
    }
    if (state.echo_request) {
      gpiod_line_request_release(state.echo_request);
      state.echo_request = nullptr;
    }
  }

  void drain_edge_events(SensorState & state)
  {
    if (!state.echo_request) {
      return;
    }
    auto * buffer = gpiod_edge_event_buffer_new(8);
    if (!buffer) {
      return;
    }
    while (gpiod_line_request_wait_edge_events(state.echo_request, 0) > 0) {
      const int ret = gpiod_line_request_read_edge_events(state.echo_request, buffer, 8);
      if (ret <= 0) {
        break;
      }
    }
    gpiod_edge_event_buffer_free(buffer);
  }

  bool wait_for_event(SensorState & state, gpiod_edge_event_type wanted, int64_t timeout_ns, uint64_t & timestamp_ns)
  {
    auto * buffer = gpiod_edge_event_buffer_new(4);
    if (!buffer) {
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(timeout_ns);
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - std::chrono::steady_clock::now()).count();
      const int ready = gpiod_line_request_wait_edge_events(state.echo_request, remaining);
      if (ready <= 0) {
        break;
      }
      const int count = gpiod_line_request_read_edge_events(state.echo_request, buffer, 4);
      if (count <= 0) {
        break;
      }
      for (int i = 0; i < count; ++i) {
        gpiod_edge_event * event = gpiod_edge_event_buffer_get_event(buffer, static_cast<unsigned long>(i));
        if (!event) {
          continue;
        }
        if (gpiod_edge_event_get_line_offset(event) != state.echo_offset) {
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

  bool setup_sensor(SensorState & state, const std::string & name)
  {
    state.trig_offset = static_cast<unsigned int>(state.config.trig_pin);
    state.echo_offset = static_cast<unsigned int>(state.config.echo_pin);
    state.trig_request = request_output_line(gpio_chip_path_, state.trig_offset, ("smart_follower_" + name + "_trig").c_str());
    if (!state.trig_request) {
      return false;
    }
    state.echo_request = request_echo_line(gpio_chip_path_, state.echo_offset, ("smart_follower_" + name + "_echo").c_str());
    if (!state.echo_request) {
      release_sensor(state);
      return false;
    }
    drain_edge_events(state);
    return true;
  }

  bool setup_gpiod_backend(const rclcpp::Logger & logger)
  {
    gpio_chip_path_ = detect_gpio_chip_path();
    if (gpio_chip_path_.empty()) {
      RCLCPP_ERROR(logger, "Failed to detect GPIO chip for libgpiod");
      return false;
    }
    if (!setup_sensor(left_, "left")) {
      RCLCPP_ERROR(logger, "Failed to request left ultrasonic GPIO lines via libgpiod");
      release_sensor(left_);
      release_sensor(right_);
      return false;
    }
    if (!setup_sensor(right_, "right")) {
      RCLCPP_ERROR(logger, "Failed to request right ultrasonic GPIO lines via libgpiod");
      release_sensor(left_);
      release_sensor(right_);
      return false;
    }
    RCLCPP_INFO(logger, "libgpiod backend ready on %s", gpio_chip_path_.c_str());
    return true;
  }
#endif

  void release_backend()
  {
#ifdef HAVE_LIBGPIOD
    release_sensor(left_);
    release_sensor(right_);
#endif
    gpio_chip_path_.clear();
  }

  void reset_measurement_state(bool clear_pending)
  {
    left_.history.clear();
    right_.history.clear();
    gpio_ok_ = false;
    gpio_backend_ = "dry";
    last_left_range_ = std::numeric_limits<double>::infinity();
    last_right_range_ = std::numeric_limits<double>::infinity();
    last_left_measure_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_right_measure_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    measure_left_next_ = true;
    if (clear_pending) {
      reconfigure_pending_ = false;
    }
  }

  void reinitialize(const rclcpp::Logger & logger)
  {
    sync_sensor_configs();
    release_backend();
    reset_measurement_state(false);
#ifdef HAVE_LIBGPIOD
    gpio_ok_ = setup_gpiod_backend(logger);
    gpio_backend_ = gpio_ok_ ? "libgpiod" : "dry";
#else
    gpio_ok_ = false;
    gpio_backend_ = "dry";
#endif
    if (!gpio_ok_) {
      RCLCPP_WARN(logger, "No usable GPIO backend available, ultrasonic node will publish inf ranges.");
    }
  }

  double read_distance_m(SensorState & state)
  {
#ifdef HAVE_LIBGPIOD
    if (!gpio_ok_ || !state.trig_request || !state.echo_request) {
      return std::numeric_limits<double>::infinity();
    }
    constexpr int64_t kTimeoutNs = 30000000LL;
    drain_edge_events(state);
    if (gpiod_line_request_set_value(state.trig_request, state.trig_offset, GPIOD_LINE_VALUE_INACTIVE) < 0) {
      return std::numeric_limits<double>::infinity();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(2));
    if (gpiod_line_request_set_value(state.trig_request, state.trig_offset, GPIOD_LINE_VALUE_ACTIVE) < 0) {
      return std::numeric_limits<double>::infinity();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(15));
    if (gpiod_line_request_set_value(state.trig_request, state.trig_offset, GPIOD_LINE_VALUE_INACTIVE) < 0) {
      return std::numeric_limits<double>::infinity();
    }

    uint64_t rise_ns = 0;
    uint64_t fall_ns = 0;
    if (!wait_for_event(state, GPIOD_EDGE_EVENT_RISING_EDGE, kTimeoutNs, rise_ns)) {
      return std::numeric_limits<double>::infinity();
    }
    if (!wait_for_event(state, GPIOD_EDGE_EVENT_FALLING_EDGE, kTimeoutNs, fall_ns)) {
      return std::numeric_limits<double>::infinity();
    }
    if (fall_ns <= rise_ns) {
      return std::numeric_limits<double>::infinity();
    }
    const double pulse_us = static_cast<double>(fall_ns - rise_ns) / 1000.0;
    return pulse_us * 0.0001715;
#else
    (void)state;
    return std::numeric_limits<double>::infinity();
#endif
  }

  double median_filtered(SensorState & state, double sample)
  {
    if (!std::isfinite(sample)) {
      return std::numeric_limits<double>::infinity();
    }
    state.history.push_back(sample);
    while (static_cast<int>(state.history.size()) > config_.window_size) {
      state.history.pop_front();
    }
    std::vector<double> tmp(state.history.begin(), state.history.end());
    std::sort(tmp.begin(), tmp.end());
    return tmp[tmp.size() / 2];
  }

  void measure_step(const rclcpp::Time & now_time)
  {
    if (measure_left_next_) {
      last_left_range_ = median_filtered(left_, read_distance_m(left_));
      last_left_measure_stamp_ = now_time;
    } else {
      last_right_range_ = median_filtered(right_, read_distance_m(right_));
      last_right_measure_stamp_ = now_time;
    }
    measure_left_next_ = !measure_left_next_;
  }

  UltrasonicRuntimeSnapshot snapshot(const rclcpp::Time & now_time) const
  {
    UltrasonicRuntimeSnapshot out;
    out.gpio_ok = gpio_ok_;
    out.gpio_backend = gpio_backend_;
    out.gpio_chip_path = gpio_chip_path_;
    out.last_left_range = last_left_range_;
    out.last_right_range = last_right_range_;
    out.last_left_age_s = last_left_measure_stamp_.nanoseconds() > 0 ? (now_time - last_left_measure_stamp_).seconds() : -1.0;
    out.last_right_age_s = last_right_measure_stamp_.nanoseconds() > 0 ? (now_time - last_right_measure_stamp_).seconds() : -1.0;
    out.measure_left_next = measure_left_next_;
    out.left_samples = static_cast<int>(left_.history.size());
    out.right_samples = static_cast<int>(right_.history.size());
    return out;
  }
};

UltrasonicRuntime::UltrasonicRuntime()
: impl_(std::make_unique<Impl>())
{
}

UltrasonicRuntime::~UltrasonicRuntime() = default;
UltrasonicRuntime::UltrasonicRuntime(UltrasonicRuntime &&) noexcept = default;
UltrasonicRuntime & UltrasonicRuntime::operator=(UltrasonicRuntime &&) noexcept = default;

void UltrasonicRuntime::set_config(const UltrasonicRuntimeConfig & config)
{
  impl_->config_ = config;
  impl_->sync_sensor_configs();
}

const UltrasonicRuntimeConfig & UltrasonicRuntime::config() const
{
  return impl_->config_;
}

void UltrasonicRuntime::clear()
{
  impl_->release_backend();
  impl_->reset_measurement_state(true);
  impl_->sync_sensor_configs();
}

void UltrasonicRuntime::reinitialize(const rclcpp::Logger & logger)
{
  impl_->reinitialize(logger);
}

void UltrasonicRuntime::request_reconfigure()
{
  impl_->reconfigure_pending_ = true;
}

bool UltrasonicRuntime::consume_reconfigure_request()
{
  const bool pending = impl_->reconfigure_pending_;
  impl_->reconfigure_pending_ = false;
  return pending;
}

void UltrasonicRuntime::measure_step(const rclcpp::Time & now_time)
{
  impl_->measure_step(now_time);
}

double UltrasonicRuntime::last_left_range() const
{
  return impl_->last_left_range_;
}

double UltrasonicRuntime::last_right_range() const
{
  return impl_->last_right_range_;
}

rclcpp::Time UltrasonicRuntime::last_left_stamp() const
{
  return impl_->last_left_measure_stamp_;
}

rclcpp::Time UltrasonicRuntime::last_right_stamp() const
{
  return impl_->last_right_measure_stamp_;
}

const std::string & UltrasonicRuntime::gpio_backend() const
{
  return impl_->gpio_backend_;
}

UltrasonicRuntimeSnapshot UltrasonicRuntime::snapshot(const rclcpp::Time & now_time) const
{
  return impl_->snapshot(now_time);
}

}  // namespace smart_follower_control
