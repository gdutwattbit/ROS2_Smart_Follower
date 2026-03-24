#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>
#include <smart_follower_msgs/msg/tracked_person.hpp>
#include <std_msgs/msg/header.hpp>

#include "smart_follower_perception/geometry_utils.hpp"
#include "smart_follower_perception/runtime.hpp"
#include "smart_follower_perception/tracker.hpp"
#include "smart_follower_perception/tracking_utils.hpp"

namespace smart_follower_perception
{

using Image = sensor_msgs::msg::Image;

struct SynchronizedFrame
{
  Image::SharedPtr color;
  Image::SharedPtr depth;
};

struct DetectionWorkItem
{
  SynchronizedFrame frame;
  bool run_detect{false};
};

struct DetectionWorkResult
{
  std::chrono::steady_clock::time_point processing_begin{};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std_msgs::msg::Header color_header;
  std_msgs::msg::Header depth_header;
  Image::SharedPtr depth_frame;
  cv::Size image_size;
  bool run_detect{false};
  double cv_bridge_ms{0.0};
  double yolo_ms{0.0};
  double yolo_preprocess_ms{0.0};
  double yolo_run_ms{0.0};
  double yolo_postprocess_ms{0.0};
  double reid_ms{0.0};
  double reid_preprocess_ms{0.0};
  double reid_run_ms{0.0};
  std::vector<Detection> detections;
  std::string reid_dim_error;
};

struct MessageBuildStats
{
  double position_projection_ms{0.0};
  double message_fill_ms{0.0};
  std::size_t position_success_count{0};
  std::size_t position_failure_count{0};
  std::size_t depth_samples_valid{0};
  double last_valid_depth_m{-1.0};
};

struct PersonPoseBuildResult
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  MessageBuildStats stats;
};

bool run_detection_work_item(
  const DetectionWorkItem & item,
  YoloDetector & yolo,
  ReidExtractor & reid,
  DetectionWorkResult & result,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

smart_follower_msgs::msg::TrackedPerson track_to_message(
  const Track & track,
  const cv::Mat & depth_image,
  const cv::Size & image_size,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  const DepthPositionConfig & depth_config,
  MessageBuildStats * stats);

PersonPoseBuildResult build_person_pose_array(
  const std::unordered_map<int, Track> & tracks,
  const std_msgs::msg::Header & color_header,
  const cv::Mat & depth_image,
  const cv::Size & image_size,
  int lock_id,
  uint8_t lock_state,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  const DepthPositionConfig & depth_config,
  const std::string & base_frame);

}  // namespace smart_follower_perception
