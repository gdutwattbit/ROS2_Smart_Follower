#include "smart_follower_perception/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace smart_follower_perception
{

std::optional<geometry_msgs::msg::Point> estimate_person_position_from_bbox(
  const cv::Rect2f & bbox,
  const cv::Size & image_size,
  const MonocularPositionConfig & config)
{
  if (image_size.width <= 0 || image_size.height <= 0 || bbox.height <= 1.0F) {
    return std::nullopt;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double fov_deg = std::clamp(static_cast<double>(config.horizontal_fov_deg), 1.0, 179.0);
  const double focal_px = (0.5 * static_cast<double>(image_size.width)) /
    std::tan(0.5 * fov_deg * kPi / 180.0);
  if (!std::isfinite(focal_px) || focal_px <= 1e-6) {
    return std::nullopt;
  }

  const double raw_forward = focal_px * static_cast<double>(config.person_height_m) /
    std::max(1.0, static_cast<double>(bbox.height));
  if (!std::isfinite(raw_forward)) {
    return std::nullopt;
  }

  const double forward = std::clamp(
    raw_forward,
    static_cast<double>(config.min_range_m),
    static_cast<double>(config.max_range_m));
  const double center_x = static_cast<double>(bbox.x + bbox.width * 0.5F);
  const double pixel_offset_x = center_x - static_cast<double>(image_size.width) * 0.5;
  const double lateral = -forward * pixel_offset_x / focal_px;

  geometry_msgs::msg::Point point;
  point.x = forward + static_cast<double>(config.camera_x_offset_m);
  point.y = lateral + static_cast<double>(config.camera_y_offset_m);
  point.z = 0.0;
  return point;
}

}  // namespace smart_follower_perception
