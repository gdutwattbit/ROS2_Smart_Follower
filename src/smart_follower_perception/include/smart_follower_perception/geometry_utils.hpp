#pragma once

#include <optional>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

namespace smart_follower_perception
{

struct MonocularPositionConfig
{
  std::string camera_info_service{"/camera/get_camera_info"};
  float camera_x_offset_m{0.0F};
  float camera_y_offset_m{0.0F};
};

struct DepthPositionConfig
{
  float min_range_m{0.2F};
  float max_range_m{4.0F};
  int sample_window_px{5};
  int min_valid_samples{3};
};

struct MonocularCameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int image_width{0};
  int image_height{0};
  bool ready{false};
};

struct DepthSampleResult
{
  float depth_m{0.0F};
  int valid_samples{0};
  bool valid{false};
};

bool is_valid_camera_intrinsics(const MonocularCameraIntrinsics & intrinsics);


DepthSampleResult sample_depth_from_bbox(
  const cv::Mat & depth,
  const cv::Rect2f & bbox,
  const DepthPositionConfig & config);

std::optional<geometry_msgs::msg::Point> estimate_person_position_from_depth_bbox(
  const cv::Rect2f & bbox,
  const cv::Mat & depth,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  const DepthPositionConfig & depth_config,
  DepthSampleResult * sample = nullptr);

}  // namespace smart_follower_perception
