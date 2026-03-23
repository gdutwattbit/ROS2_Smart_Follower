#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/core.hpp>

#include "smart_follower_perception/geometry_utils.hpp"

using smart_follower_perception::MonocularCameraIntrinsics;
using smart_follower_perception::MonocularPositionConfig;
using smart_follower_perception::estimate_person_position_from_bbox;
using smart_follower_perception::is_valid_camera_intrinsics;
using smart_follower_perception::make_fallback_camera_intrinsics;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(GeometryUtils, FallbackIntrinsicsUsesImageCenter)
{
  MonocularPositionConfig config;
  config.horizontal_fov_deg = 69.0F;

  const auto intrinsics = make_fallback_camera_intrinsics(cv::Size(640, 480), config);
  ASSERT_TRUE(is_valid_camera_intrinsics(intrinsics));
  EXPECT_NEAR(intrinsics.cx, 319.5, 1e-6);
  EXPECT_NEAR(intrinsics.cy, 239.5, 1e-6);
  EXPECT_GT(intrinsics.fx, 0.0);
  EXPECT_NEAR(intrinsics.fx, intrinsics.fy, 1e-6);
}

TEST(GeometryUtils, ProjectsBottomCenterToFiniteForwardPosition)
{
  MonocularCameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;
  intrinsics.image_width = 640;
  intrinsics.image_height = 480;
  intrinsics.ready = true;

  MonocularPositionConfig config;
  config.camera_height_m = 0.30F;
  config.camera_pitch_deg = 18.0F;
  config.min_downward_angle_deg = 2.0F;
  config.min_range_m = 0.1F;
  config.max_range_m = 6.0F;

  const cv::Rect2f bbox(280.0F, 180.0F, 80.0F, 200.0F);
  const auto point = estimate_person_position_from_bbox(bbox, intrinsics, config);
  ASSERT_TRUE(point.has_value());
  EXPECT_TRUE(std::isfinite(point->x));
  EXPECT_TRUE(std::isfinite(point->y));
  EXPECT_GT(point->x, 0.0);
  EXPECT_NEAR(point->y, 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(point->z, 0.0);
}

TEST(GeometryUtils, LeftAndRightBottomPointsProduceExpectedLateralSign)
{
  MonocularCameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;
  intrinsics.image_width = 640;
  intrinsics.image_height = 480;
  intrinsics.ready = true;

  MonocularPositionConfig config;
  config.camera_height_m = 0.30F;
  config.camera_pitch_deg = 18.0F;
  config.min_downward_angle_deg = 2.0F;
  config.min_range_m = 0.1F;
  config.max_range_m = 6.0F;

  const auto left = estimate_person_position_from_bbox(
    cv::Rect2f(120.0F, 180.0F, 80.0F, 200.0F), intrinsics, config);
  const auto right = estimate_person_position_from_bbox(
    cv::Rect2f(440.0F, 180.0F, 80.0F, 200.0F), intrinsics, config);

  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_GT(left->y, 0.0);
  EXPECT_LT(right->y, 0.0);
}

TEST(GeometryUtils, RejectsProjectionNearHorizon)
{
  MonocularCameraIntrinsics intrinsics;
  intrinsics.fx = 500.0;
  intrinsics.fy = 500.0;
  intrinsics.cx = 320.0;
  intrinsics.cy = 240.0;
  intrinsics.image_width = 640;
  intrinsics.image_height = 480;
  intrinsics.ready = true;

  MonocularPositionConfig config;
  config.camera_height_m = 0.30F;
  config.camera_pitch_deg = 18.0F;
  config.min_downward_angle_deg = 2.0F;

  const float bottom_v = static_cast<float>(
    intrinsics.cy - intrinsics.fy * std::tan((18.0 - 1.0) * kPi / 180.0));
  const auto point = estimate_person_position_from_bbox(
    cv::Rect2f(280.0F, bottom_v - 120.0F, 80.0F, 120.0F),
    intrinsics,
    config);
  EXPECT_FALSE(point.has_value());
}
