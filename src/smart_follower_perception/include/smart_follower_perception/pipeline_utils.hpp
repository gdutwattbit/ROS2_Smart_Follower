#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>
#include <smart_follower_msgs/msg/tracked_person.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>

#include "smart_follower_perception/runtime.hpp"
#include "smart_follower_perception/tracker.hpp"
#include "smart_follower_perception/tracking_utils.hpp"

namespace smart_follower_perception
{

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

struct SynchronizedFrame
{
  Image::SharedPtr color;
  Image::SharedPtr depth;
  CameraInfo::SharedPtr info;
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
  CameraInfo::SharedPtr camera_info;
  cv::Size image_size;
  bool run_detect{false};
  double cv_bridge_ms{0.0};
  double yolo_ms{0.0};
  double yolo_preprocess_ms{0.0};
  double yolo_run_ms{0.0};
  double yolo_postprocess_ms{0.0};
  double depth_ms{0.0};
  double reid_ms{0.0};
  double reid_preprocess_ms{0.0};
  double reid_run_ms{0.0};
  std::vector<Detection> detections;
  std::string reid_dim_error;
};

struct MessageBuildStats
{
  double tf_lookup_ms{0.0};
  double tf_transform_ms{0.0};
  double message_fill_ms{0.0};
  std::size_t tf_success_count{0};
  std::size_t tf_failure_count{0};
};

struct PersonPoseBuildResult
{
  smart_follower_msgs::msg::PersonPoseArray msg;
  MessageBuildStats stats;
};

bool is_sync_pair_within_slop(
  const Image::SharedPtr & color_msg,
  const Image::SharedPtr & depth_msg,
  double sync_slop_sec);

bool run_detection_work_item(
  const DetectionWorkItem & item,
  YoloDetector & yolo,
  ReidExtractor & reid,
  float depth_min_m,
  float depth_max_m,
  DetectionWorkResult & result,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

smart_follower_msgs::msg::TrackedPerson track_to_message(
  const Track & track,
  image_geometry::PinholeCameraModel & camera_model,
  const geometry_msgs::msg::TransformStamped * camera_to_base_tf,
  float depth_min_m,
  float depth_max_m,
  MessageBuildStats * stats);

PersonPoseBuildResult build_person_pose_array(
  const std::unordered_map<int, Track> & tracks,
  const std_msgs::msg::Header & color_header,
  const std_msgs::msg::Header & depth_header,
  int lock_id,
  uint8_t lock_state,
  image_geometry::PinholeCameraModel & camera_model,
  tf2_ros::Buffer & tf_buffer,
  const std::string & base_frame,
  float depth_min_m,
  float depth_max_m,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock);

}  // namespace smart_follower_perception
