#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <opencv2/core.hpp>

#include "smart_follower_perception/geometry_utils.hpp"

using smart_follower_perception::DepthPositionConfig;
using smart_follower_perception::DepthSampleResult;
using smart_follower_perception::CameraIntrinsics;
using smart_follower_perception::CameraConfig;
using smart_follower_perception::estimate_person_position_from_depth_bbox;
using smart_follower_perception::is_valid_camera_intrinsics;
using smart_follower_perception::sample_depth_from_bbox;

namespace
{

CameraIntrinsics make_intrinsics()
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;
  intrinsics.image_width = 640;
  intrinsics.image_height = 480;
  intrinsics.ready = true;
  return intrinsics;
}

CameraConfig make_camera_config()
{
  CameraConfig config;
  config.x_offset_m = 0.175F;
  config.y_offset_m = 0.01F;
  return config;
}

}  // namespace

TEST(GeometryUtils, CameraIntrinsicsValidationRejectsInvalidValues)
{
  auto intrinsics = make_intrinsics();
  EXPECT_TRUE(is_valid_camera_intrinsics(intrinsics));

  intrinsics.fx = 0.0;
  EXPECT_FALSE(is_valid_camera_intrinsics(intrinsics));

  intrinsics = make_intrinsics();
  intrinsics.ready = false;
  EXPECT_FALSE(is_valid_camera_intrinsics(intrinsics));
}

TEST(GeometryUtils, DepthSamplingUsesMedianAndIgnoresInvalidValues)
{
  DepthPositionConfig config;
  config.min_range_m = 0.2F;
  config.max_range_m = 4.0F;
  config.sample_window_px = 9;
  config.min_valid_samples = 5;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const cv::Rect2f bbox(280.0F, 160.0F, 80.0F, 200.0F);
  const int center_x = 320;
  const int center_y = 296;
  const std::uint16_t values[9] = {1500, 0, 1550, 1600, 1580, 40000, 1520, 1510, 1540};
  int idx = 0;
  for (int y = center_y - 1; y <= center_y + 1; ++y) {
    for (int x = center_x - 1; x <= center_x + 1; ++x) {
      depth.at<std::uint16_t>(y, x) = values[idx++];
    }
  }

  const auto sample = sample_depth_from_bbox(depth, bbox, config);
  EXPECT_TRUE(sample.valid);
  EXPECT_EQ(sample.valid_samples, 7);
  EXPECT_NEAR(sample.depth_m, 1.54F, 1e-3F);
}

TEST(GeometryUtils, DepthPositionRejectsInsufficientValidSamples)
{
  const auto intrinsics = make_intrinsics();
  auto camera = make_camera_config();
  DepthPositionConfig depth_config;
  depth_config.min_range_m = 0.2F;
  depth_config.max_range_m = 4.0F;
  depth_config.sample_window_px = 9;
  depth_config.min_valid_samples = 4;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const cv::Rect2f bbox(280.0F, 160.0F, 80.0F, 200.0F);
  depth.at<std::uint16_t>(296, 320) = 1500;
  depth.at<std::uint16_t>(297, 320) = 1520;
  depth.at<std::uint16_t>(295, 320) = 1510;

  DepthSampleResult sample;
  const auto point = estimate_person_position_from_depth_bbox(
    bbox,
    depth,
    intrinsics,
    camera,
    depth_config,
    &sample);

  EXPECT_FALSE(point.has_value());
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.valid_samples, 3);
}

TEST(GeometryUtils, DepthPositionProducesExpectedLateralSign)
{
  const auto intrinsics = make_intrinsics();
  auto camera = make_camera_config();
  camera.x_offset_m = 0.175F;
  camera.y_offset_m = 0.01F;

  DepthPositionConfig depth_config;
  depth_config.min_range_m = 0.2F;
  depth_config.max_range_m = 4.0F;
  depth_config.sample_window_px = 9;
  depth_config.min_valid_samples = 5;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1500));
  const auto left = estimate_person_position_from_depth_bbox(
    cv::Rect2f(120.0F, 160.0F, 80.0F, 200.0F), depth, intrinsics, camera, depth_config);
  const auto right = estimate_person_position_from_depth_bbox(
    cv::Rect2f(440.0F, 160.0F, 80.0F, 200.0F), depth, intrinsics, camera, depth_config);

  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_NEAR(left->x, 1.675, 1e-3);
  EXPECT_NEAR(right->x, 1.675, 1e-3);
  EXPECT_GT(left->y, camera.y_offset_m);
  EXPECT_LT(right->y, camera.y_offset_m);
}
TEST(GeometryUtils, DepthSamplingRecoversUsingLowerBodyWindowsWhenTopIsCutOff)
{
  DepthPositionConfig config;
  config.min_range_m = 0.2F;
  config.max_range_m = 4.0F;
  config.sample_window_px = 9;
  config.min_valid_samples = 5;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const cv::Rect2f bbox(260.0F, 0.0F, 120.0F, 340.0F);
  const int center_x = 320;
  const int lower_center_y = static_cast<int>(std::lround(bbox.y + bbox.height * 0.78F));
  const std::uint16_t values[9] = {1660, 0, 1640, 1650, 1670, 0, 1630, 1680, 1665};
  int idx = 0;
  for (int y = lower_center_y - 1; y <= lower_center_y + 1; ++y) {
    for (int x = center_x - 1; x <= center_x + 1; ++x) {
      depth.at<std::uint16_t>(y, x) = values[idx++];
    }
  }

  const auto sample = sample_depth_from_bbox(depth, bbox, config);
  EXPECT_TRUE(sample.valid);
  EXPECT_EQ(sample.valid_samples, 7);
  EXPECT_NEAR(sample.depth_m, 1.66F, 1e-3F);
}
