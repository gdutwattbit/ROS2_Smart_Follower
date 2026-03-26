#include <gtest/gtest.h>

#include <cstdint>

#include <opencv2/core.hpp>

#include "smart_follower_perception/geometry_utils.hpp"

using smart_follower_perception::DepthPositionConfig;
using smart_follower_perception::DepthSampleResult;
using smart_follower_perception::MonocularCameraIntrinsics;
using smart_follower_perception::MonocularPositionConfig;
using smart_follower_perception::estimate_person_position_from_depth_bbox;
using smart_follower_perception::is_valid_camera_intrinsics;
using smart_follower_perception::sample_depth_from_bbox;

namespace
{

MonocularCameraIntrinsics make_intrinsics()
{
  MonocularCameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;
  intrinsics.image_width = 640;
  intrinsics.image_height = 480;
  intrinsics.ready = true;
  return intrinsics;
}

MonocularPositionConfig make_monocular_config()
{
  MonocularPositionConfig config;
  config.camera_x_offset_m = 0.175F;
  config.camera_y_offset_m = 0.01F;
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
  config.sample_window_px = 5;
  config.min_valid_samples = 3;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const cv::Rect2f bbox(280.0F, 160.0F, 80.0F, 200.0F);
  const int center_x = 320;
  const int center_y = 300;
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
  auto monocular = make_monocular_config();
  DepthPositionConfig depth_config;
  depth_config.min_range_m = 0.2F;
  depth_config.max_range_m = 4.0F;
  depth_config.sample_window_px = 5;
  depth_config.min_valid_samples = 4;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const cv::Rect2f bbox(280.0F, 160.0F, 80.0F, 200.0F);
  depth.at<std::uint16_t>(300, 320) = 1500;
  depth.at<std::uint16_t>(301, 320) = 1520;
  depth.at<std::uint16_t>(299, 320) = 1510;

  DepthSampleResult sample;
  const auto point = estimate_person_position_from_depth_bbox(
    bbox,
    depth,
    intrinsics,
    monocular,
    depth_config,
    &sample);

  EXPECT_FALSE(point.has_value());
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.valid_samples, 3);
}

TEST(GeometryUtils, DepthPositionProducesExpectedLateralSign)
{
  const auto intrinsics = make_intrinsics();
  auto monocular = make_monocular_config();
  monocular.camera_x_offset_m = 0.175F;
  monocular.camera_y_offset_m = 0.01F;

  DepthPositionConfig depth_config;
  depth_config.min_range_m = 0.2F;
  depth_config.max_range_m = 4.0F;
  depth_config.sample_window_px = 5;
  depth_config.min_valid_samples = 3;

  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1500));
  const auto left = estimate_person_position_from_depth_bbox(
    cv::Rect2f(120.0F, 160.0F, 80.0F, 200.0F), depth, intrinsics, monocular, depth_config);
  const auto right = estimate_person_position_from_depth_bbox(
    cv::Rect2f(440.0F, 160.0F, 80.0F, 200.0F), depth, intrinsics, monocular, depth_config);

  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_NEAR(left->x, 1.675, 1e-3);
  EXPECT_NEAR(right->x, 1.675, 1e-3);
  EXPECT_GT(left->y, monocular.camera_y_offset_m);
  EXPECT_LT(right->y, monocular.camera_y_offset_m);
}
