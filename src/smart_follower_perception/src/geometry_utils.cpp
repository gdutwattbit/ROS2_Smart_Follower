#include "smart_follower_perception/geometry_utils.hpp"

#include <algorithm>
#include <cmath>

namespace smart_follower_perception
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

}  // namespace

bool is_valid_camera_intrinsics(const MonocularCameraIntrinsics & intrinsics)
{
  return intrinsics.ready &&
         std::isfinite(intrinsics.fx) && intrinsics.fx > 1e-6 &&
         std::isfinite(intrinsics.fy) && intrinsics.fy > 1e-6 &&
         std::isfinite(intrinsics.cx) &&
         std::isfinite(intrinsics.cy) &&
         intrinsics.image_width > 0 &&
         intrinsics.image_height > 0;
}

MonocularCameraIntrinsics make_fallback_camera_intrinsics(
  const cv::Size & image_size,
  const MonocularPositionConfig & config)
{
  MonocularCameraIntrinsics intrinsics;
  if (image_size.width <= 0 || image_size.height <= 0) {
    return intrinsics;
  }

  const double fov_deg = std::clamp(static_cast<double>(config.horizontal_fov_deg), 1.0, 179.0);
  const double focal_px = (0.5 * static_cast<double>(image_size.width)) /
    std::tan(0.5 * deg_to_rad(fov_deg));
  if (!std::isfinite(focal_px) || focal_px <= 1e-6) {
    return intrinsics;
  }

  intrinsics.fx = focal_px;
  intrinsics.fy = focal_px;
  intrinsics.cx = (static_cast<double>(image_size.width) - 1.0) * 0.5;
  intrinsics.cy = (static_cast<double>(image_size.height) - 1.0) * 0.5;
  intrinsics.image_width = image_size.width;
  intrinsics.image_height = image_size.height;
  intrinsics.ready = true;
  return intrinsics;
}

std::optional<geometry_msgs::msg::Point> estimate_person_position_from_bbox(
  const cv::Rect2f & bbox,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & config)
{
  if (
    bbox.width <= 1.0F || bbox.height <= 1.0F ||
    !is_valid_camera_intrinsics(intrinsics) ||
    !std::isfinite(config.camera_height_m) || config.camera_height_m <= 1e-6F)
  {
    return std::nullopt;
  }

  const double u = static_cast<double>(bbox.x + bbox.width * 0.5F);
  const double v = static_cast<double>(bbox.y + bbox.height);
  const double du = (u - intrinsics.cx) / intrinsics.fx;
  const double dv = (v - intrinsics.cy) / intrinsics.fy;
  const double phi = std::atan(dv);
  const double gamma = deg_to_rad(static_cast<double>(config.camera_pitch_deg)) + phi;
  const double min_downward_angle = deg_to_rad(
    std::max(0.0, static_cast<double>(config.min_downward_angle_deg)));

  if (!std::isfinite(gamma) || gamma <= min_downward_angle) {
    return std::nullopt;
  }

  const double raw_forward = static_cast<double>(config.camera_height_m) / std::tan(gamma);
  if (!std::isfinite(raw_forward) || raw_forward <= 0.0) {
    return std::nullopt;
  }

  const double forward = std::clamp(
    raw_forward,
    static_cast<double>(config.min_range_m),
    static_cast<double>(config.max_range_m));
  const double lateral = -forward * du;

  geometry_msgs::msg::Point point;
  point.x = forward + static_cast<double>(config.camera_x_offset_m);
  point.y = lateral + static_cast<double>(config.camera_y_offset_m);
  point.z = 0.0;
  return point;
}

}  // namespace smart_follower_perception
