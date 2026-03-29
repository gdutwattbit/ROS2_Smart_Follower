#include "smart_follower_perception/geometry_utils.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

namespace smart_follower_perception
{

namespace
{
constexpr std::array<float, 4> kDepthSampleYRatios{0.78F, 0.68F, 0.58F, 0.48F};


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

void append_window_samples(
  const cv::Mat & depth,
  int center_x,
  int center_y,
  const DepthPositionConfig & config,
  std::vector<float> & valid)
{
  const int side = std::max(1, config.sample_window_px | 1);
  const int radius = side / 2;

  const int x0 = std::max(0, center_x - radius);
  const int y0 = std::max(0, center_y - radius);
  const int x1 = std::min(depth.cols - 1, center_x + radius);
  const int y1 = std::min(depth.rows - 1, center_y + radius);
  if (x0 > x1 || y0 > y1) {
    return;
  }

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
}

}  // namespace

bool is_valid_camera_intrinsics(const CameraIntrinsics & intrinsics)
{
  return intrinsics.ready &&
         std::isfinite(intrinsics.fx) && intrinsics.fx > 1e-6 &&
         std::isfinite(intrinsics.fy) && intrinsics.fy > 1e-6 &&
         std::isfinite(intrinsics.cx) &&
         std::isfinite(intrinsics.cy) &&
         intrinsics.image_width > 0 &&
         intrinsics.image_height > 0;
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
  std::vector<float> valid;
  const int side = std::max(1, config.sample_window_px | 1);
  valid.reserve(static_cast<std::size_t>(side * side * static_cast<int>(kDepthSampleYRatios.size())));
  for (const float ratio : kDepthSampleYRatios) {
    const int center_y = static_cast<int>(std::lround(bbox.y + bbox.height * ratio));
    append_window_samples(depth, center_x, center_y, config, valid);
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
  const CameraIntrinsics & intrinsics,
  const CameraConfig & camera,
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
  point.x = forward + static_cast<double>(camera.x_offset_m);
  point.y = (-forward * du) + static_cast<double>(camera.y_offset_m);
  point.z = 0.0;

  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return std::nullopt;
  }

  return point;
}

}  // namespace smart_follower_perception
