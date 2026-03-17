#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>

namespace smart_follower_perception
{

float sample_depth_m(
  const cv::Mat & depth,
  int cx,
  int cy,
  float depth_min_m,
  float depth_max_m);

std::optional<geometry_msgs::msg::Point> pixel_to_base_point(
  const cv::Rect2f & bbox,
  float depth_m,
  const std_msgs::msg::Header & header,
  const image_geometry::PinholeCameraModel & camera_model,
  tf2_ros::Buffer & tf_buffer,
  const std::string & base_frame,
  float depth_min_m,
  float depth_max_m,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

}  // namespace smart_follower_perception
