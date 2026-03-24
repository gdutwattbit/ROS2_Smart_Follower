#include "smart_follower_perception/geometry_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

namespace smart_follower_perception
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kDepthSampleYRatio = 0.70F;

double deg_to_rad(double deg)
{
  return deg * kPi / 180.0;
}

bool extract_depth_meters(const cv::Mat & depth, int x, int y, float & meters)
{
  if (depth.type() == CV_16UC1) {
    const auto mm = depth.at<uint16_t>(y, x);
    if (mm == 0U) {
      return false;
    }
    meters = static_cast<float>(mm) * 0.001F;
    return true;
  }

  if (depth.type() == CV_32FC1) {
    meters = depth.at<float>(y, x);
    return true;
  }

  return false;
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

DepthSampleResult sample_depth_from_bbox(
  const cv::Mat & depth,
  const cv::Rect2f & bbox,
  const DepthPositionConfig & config)
{
  DepthSampleResult result;
  result.depth_m = std::numeric_limits<float>::quiet_NaN();

  if (depth.empty() || bbox.width <= 1.0F || bbox.height <= 1.0F) {
    return result;
  }

  const int center_x = static_cast<int>(std::lround(bbox.x + bbox.width * 0.5F));
  const int center_y = static_cast<int>(std::lround(bbox.y + bbox.height * kDepthSampleYRatio));
  const int side = std::max(1, config.sample_window_px | 1);
  const int radius = side / 2;

  const int x0 = std::max(0, center_x - radius);
  const int y0 = std::max(0, center_y - radius);
  const int x1 = std::min(depth.cols - 1, center_x + radius);
  const int y1 = std::min(depth.rows - 1, center_y + radius);
  if (x0 > x1 || y0 > y1) {
    return result;
  }

  std::vector<float> valid;
  valid.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      float meters = 0.0F;
      if (!extract_depth_meters(depth, x, y, meters)) {
        continue;
      }
      if (std::isfinite(meters) && meters >= config.min_range_m && meters <= config.max_range_m) {
        valid.push_back(meters);
      }
    }
  }

  result.valid_samples = static_cast<int>(valid.size());
  if (result.valid_samples < std::max(1, config.min_valid_samples)) {
    return result;
  }

  std::nth_element(valid.begin(), valid.begin() + valid.size() / 2, valid.end());
  result.depth_m = valid[valid.size() / 2];
  result.valid = std::isfinite(result.depth_m);
  return result;
}

std::optional<geometry_msgs::msg::Point> estimate_person_position_from_depth_bbox(
  const cv::Rect2f & bbox,
  const cv::Mat & depth,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  const DepthPositionConfig & depth_config,
  DepthSampleResult * sample)
{
  if (sample != nullptr) {
    *sample = DepthSampleResult{};
    sample->depth_m = std::numeric_limits<float>::quiet_NaN();
  }

  if (
    bbox.width <= 1.0F || bbox.height <= 1.0F ||
    !is_valid_camera_intrinsics(intrinsics) ||
    depth.empty())
  {
    return std::nullopt;
  }

  const auto sampled = sample_depth_from_bbox(depth, bbox, depth_config);
  if (sample != nullptr) {
    *sample = sampled;
  }
  if (!sampled.valid) {
    return std::nullopt;
  }

  const double u = static_cast<double>(bbox.x + bbox.width * 0.5F);
  const double du = (u - intrinsics.cx) / intrinsics.fx;
  const double forward = static_cast<double>(sampled.depth_m);
  if (!std::isfinite(forward) || forward < depth_config.min_range_m || forward > depth_config.max_range_m) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point point;
  point.x = forward + static_cast<double>(monocular.camera_x_offset_m);
  point.y = (-forward * du) + static_cast<double>(monocular.camera_y_offset_m);
  point.z = 0.0;

  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return std::nullopt;
  }

  return point;
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
