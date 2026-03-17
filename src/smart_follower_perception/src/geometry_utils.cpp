#include "smart_follower_perception/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace smart_follower_perception
{

float sample_depth_m(
  const cv::Mat & depth,
  int cx,
  int cy,
  float depth_min_m,
  float depth_max_m)
{
  if (depth.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const int x0 = std::max(0, cx - 2);
  const int y0 = std::max(0, cy - 2);
  const int x1 = std::min(depth.cols - 1, cx + 2);
  const int y1 = std::min(depth.rows - 1, cy + 2);

  std::vector<float> valid;
  valid.reserve(25);
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      float meters = 0.0F;
      if (depth.type() == CV_16UC1) {
        const uint16_t mm = depth.at<uint16_t>(y, x);
        if (mm == 0U) {
          continue;
        }
        meters = static_cast<float>(mm) * 0.001F;
      } else if (depth.type() == CV_32FC1) {
        meters = depth.at<float>(y, x);
      } else {
        continue;
      }

      if (std::isfinite(meters) && meters >= depth_min_m && meters <= depth_max_m) {
        valid.push_back(meters);
      }
    }
  }

  if (valid.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::nth_element(valid.begin(), valid.begin() + valid.size() / 2, valid.end());
  return valid[valid.size() / 2];
}

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
  rclcpp::Clock & clock)
{
  if (!std::isfinite(depth_m) || depth_m < depth_min_m || depth_m > depth_max_m) {
    return std::nullopt;
  }

  const double cx = static_cast<double>(bbox.x + bbox.width * 0.5F);
  const double cy = static_cast<double>(bbox.y + bbox.height * 0.5F);
  const auto ray = camera_model.projectPixelTo3dRay(cv::Point2d(cx, cy));

  geometry_msgs::msg::PointStamped cam_point;
  cam_point.header = header;
  cam_point.point.x = ray.x * depth_m;
  cam_point.point.y = ray.y * depth_m;
  cam_point.point.z = depth_m;

  try {
    const auto tf = tf_buffer.lookupTransform(base_frame, header.frame_id, header.stamp, tf2::durationFromSec(0.02));
    geometry_msgs::msg::PointStamped base_point;
    tf2::doTransform(cam_point, base_point, tf);
    return base_point.point;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger,
      clock,
      2000,
      "TF lookup failed (%s <- %s): %s",
      base_frame.c_str(),
      header.frame_id.c_str(),
      ex.what());
    return std::nullopt;
  }
}

}  // namespace smart_follower_perception
