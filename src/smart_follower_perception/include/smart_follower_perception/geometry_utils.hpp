#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

std::optional<geometry_msgs::msg::TransformStamped> lookup_camera_to_base_transform(
  const std_msgs::msg::Header & header,
  tf2_ros::Buffer & tf_buffer,
  const std::string & base_frame,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

std::optional<geometry_msgs::msg::Point> pixel_to_base_point(
  const cv::Rect2f & bbox,
  float depth_m,
  const image_geometry::PinholeCameraModel & camera_model,
  const geometry_msgs::msg::TransformStamped & camera_to_base_tf,
  float depth_min_m,
  float depth_max_m);

}  // namespace smart_follower_perception
