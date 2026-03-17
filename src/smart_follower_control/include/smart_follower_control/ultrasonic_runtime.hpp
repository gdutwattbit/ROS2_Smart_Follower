#pragma once

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

namespace smart_follower_control
{

struct UltrasonicSensorConfig
{
  int trig_pin{0};
  int echo_pin{0};
  std::string topic;
  std::string frame;
};

struct UltrasonicRuntimeConfig
{
  double rate{10.0};
  int window_size{5};
  double min_range{0.03};
  double max_range{3.0};
  UltrasonicSensorConfig left;
  UltrasonicSensorConfig right;
};

struct UltrasonicRuntimeSnapshot
{
  bool gpio_ok{false};
  std::string gpio_backend{"dry"};
  std::string gpio_chip_path;
  double last_left_range{0.0};
  double last_right_range{0.0};
  double last_left_age_s{-1.0};
  double last_right_age_s{-1.0};
  bool measure_left_next{true};
  int left_samples{0};
  int right_samples{0};
};

class UltrasonicRuntime
{
public:
  UltrasonicRuntime();
  ~UltrasonicRuntime();
  UltrasonicRuntime(UltrasonicRuntime &&) noexcept;
  UltrasonicRuntime & operator=(UltrasonicRuntime &&) noexcept;
  UltrasonicRuntime(const UltrasonicRuntime &) = delete;
  UltrasonicRuntime & operator=(const UltrasonicRuntime &) = delete;

  void set_config(const UltrasonicRuntimeConfig & config);
  const UltrasonicRuntimeConfig & config() const;

  void clear();
  void reinitialize(const rclcpp::Logger & logger);

  void request_reconfigure();
  bool consume_reconfigure_request();

  void measure_step(const rclcpp::Time & now_time);

  double last_left_range() const;
  double last_right_range() const;
  rclcpp::Time last_left_stamp() const;
  rclcpp::Time last_right_stamp() const;
  const std::string & gpio_backend() const;
  UltrasonicRuntimeSnapshot snapshot(const rclcpp::Time & now_time) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace smart_follower_control
