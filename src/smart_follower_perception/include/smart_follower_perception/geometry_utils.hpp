#pragma once

#include <optional>

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

namespace smart_follower_perception
{

struct MonocularPositionConfig
{
  float person_height_m{1.70F};
  float horizontal_fov_deg{69.0F};
  float min_range_m{0.6F};
  float max_range_m{6.0F};
  float camera_x_offset_m{0.0F};
  float camera_y_offset_m{0.0F};
};

std::optional<geometry_msgs::msg::Point> estimate_person_position_from_bbox(
  const cv::Rect2f & bbox,
  const cv::Size & image_size,
  const MonocularPositionConfig & config);

}  // namespace smart_follower_perception
