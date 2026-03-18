#include "smart_follower_control/obstacle_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <cv_bridge/cv_bridge.h>

namespace smart_follower_control
{
namespace
{
float percentile(std::vector<float> data, float q)
{
  if (data.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  q = std::max(0.0F, std::min(1.0F, q));
  std::sort(data.begin(), data.end());
  float idxf = q * static_cast<float>(data.size() - 1);
  std::size_t lo = static_cast<std::size_t>(std::floor(idxf));
  std::size_t hi = static_cast<std::size_t>(std::ceil(idxf));
  if (lo == hi) {
    return data[lo];
  }
  float t = idxf - static_cast<float>(lo);
  return data[lo] * (1.0F - t) + data[hi] * t;
}
}  // namespace

void ObstacleRuntime::set_config(const ObstacleRuntimeConfig & config)
{
  config_ = config;
}

void ObstacleRuntime::clear()
{
  left_dist_ = std::numeric_limits<double>::infinity();
  right_dist_ = std::numeric_limits<double>::infinity();
  depth_dist_ = std::numeric_limits<double>::infinity();
  current_speed_ = 0.0;
  left_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  right_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  depth_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  depth_message_count_ = 0;
  depth_process_count_ = 0;
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_msg_.reset();
}

void ObstacleRuntime::on_left_range(double range, const rclcpp::Time & stamp)
{
  left_dist_ = range;
  left_stamp_ = stamp;
}

void ObstacleRuntime::on_right_range(double range, const rclcpp::Time & stamp)
{
  right_dist_ = range;
  right_stamp_ = stamp;
}

void ObstacleRuntime::on_depth(const sensor_msgs::msg::Image::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_msg_ = msg;
  depth_message_count_ += 1;
}

void ObstacleRuntime::on_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  current_speed_ = std::abs(msg.linear.x);
}

geometry_msgs::msg::Twist ObstacleRuntime::compute_command(const rclcpp::Time & now_time)
{
  refresh_depth_from_latest();

  const auto stale = [&](const rclcpp::Time & stamp) {
    return stamp.nanoseconds() == 0 || (now_time - stamp).seconds() > 0.5;
  };

  const double left = stale(left_stamp_) ? std::numeric_limits<double>::infinity() : left_dist_;
  const double right = stale(right_stamp_) ? std::numeric_limits<double>::infinity() : right_dist_;
  const double depth = stale(depth_stamp_) ? std::numeric_limits<double>::infinity() : depth_dist_;

  const double d_safe = config_.d_min + current_speed_ * config_.t_react +
                        (current_speed_ * current_speed_) / (2.0 * std::max(1e-3, config_.a_brake)) +
                        config_.margin;
  const double d_enter = d_safe;
  const double d_exit = d_safe + config_.exit_margin;
  const double d_danger = std::max(0.18, 0.6 * d_safe);
  const double fused_min = std::min({left, right, depth});

  geometry_msgs::msg::Twist out;
  if (fused_min > d_exit) {
    return out;
  }
  if (left < d_danger && right < d_danger) {
    out.linear.x = config_.back_speed;
    return out;
  }
  if (left < d_danger && right > d_enter) {
    out.angular.z = -std::abs(config_.turn_speed);
    return out;
  }
  if (right < d_danger && left > d_enter) {
    out.angular.z = std::abs(config_.turn_speed);
    return out;
  }
  if (fused_min < d_enter) {
    out.angular.z = (left < right) ? -std::abs(config_.slow_turn_speed) : std::abs(config_.slow_turn_speed);
  }
  return out;
}

ObstacleRuntimeSnapshot ObstacleRuntime::snapshot(const rclcpp::Time & now_time) const
{
  ObstacleRuntimeSnapshot out;
  out.left_dist = left_dist_;
  out.right_dist = right_dist_;
  out.depth_dist = depth_dist_;
  out.left_age_s = left_stamp_.nanoseconds() > 0 ? std::max(0.0, (now_time - left_stamp_).seconds()) : -1.0;
  out.right_age_s = right_stamp_.nanoseconds() > 0 ? std::max(0.0, (now_time - right_stamp_).seconds()) : -1.0;
  out.depth_age_s = depth_stamp_.nanoseconds() > 0 ? std::max(0.0, (now_time - depth_stamp_).seconds()) : -1.0;
  out.depth_message_count = static_cast<int>(depth_message_count_);
  out.depth_process_count = static_cast<int>(depth_process_count_);
  out.depth_sample_stride = config_.depth_sample_stride;
  out.current_speed = current_speed_;
  return out;
}

void ObstacleRuntime::refresh_depth_from_latest()
{
  sensor_msgs::msg::Image::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    msg = latest_depth_msg_;
  }
  if (!msg) {
    return;
  }

  const rclcpp::Time msg_stamp(msg->header.stamp);
  if (msg_stamp.nanoseconds() == 0 || msg_stamp.nanoseconds() == depth_stamp_.nanoseconds()) {
    return;
  }

  cv::Mat depth;
  try {
    if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
    } else {
      depth = cv_bridge::toCvShare(msg)->image;
    }
  } catch (const cv_bridge::Exception &) {
    return;
  }

  const int w = depth.cols;
  const int h = depth.rows;
  if (w <= 0 || h <= 0) {
    return;
  }

  const int roi_w = static_cast<int>(w * config_.depth_roi_width_ratio);
  const int roi_h = static_cast<int>(h * config_.depth_roi_height_ratio);
  const int x0 = std::max(0, (w - roi_w) / 2);
  const int y0 = std::max(0, (h - roi_h) / 2);
  const int x1 = std::min(w - 1, x0 + roi_w);
  const int y1 = std::min(h - 1, y0 + roi_h);
  const int stride = std::max(1, config_.depth_sample_stride);

  std::vector<float> valid;
  valid.reserve(static_cast<std::size_t>(((roi_w + stride - 1) / stride) * ((roi_h + stride - 1) / stride)));
  for (int y = y0; y <= y1; y += stride) {
    for (int x = x0; x <= x1; x += stride) {
      float m = 0.0F;
      if (depth.type() == CV_16UC1) {
        const uint16_t mm = depth.at<uint16_t>(y, x);
        if (mm == 0) {
          continue;
        }
        m = static_cast<float>(mm) * 0.001F;
      } else if (depth.type() == CV_32FC1) {
        m = depth.at<float>(y, x);
      } else {
        continue;
      }

      if (std::isfinite(m) && m > 0.05F && m < 8.0F) {
        valid.push_back(m);
      }
    }
  }

  if (!valid.empty()) {
    depth_dist_ = percentile(valid, static_cast<float>(config_.depth_percentile));
    depth_stamp_ = msg_stamp;
    depth_process_count_ += 1;
  }
}

}  // namespace smart_follower_control
