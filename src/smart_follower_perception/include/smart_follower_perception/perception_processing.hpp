#pragma once

#include <functional>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include "smart_follower_perception/lock_manager.hpp"
#include "smart_follower_perception/perception_diagnostics.hpp"
#include "smart_follower_perception/perception_params.hpp"
#include "smart_follower_perception/pipeline_utils.hpp"
#include "smart_follower_perception/runtime.hpp"
#include "smart_follower_perception/tracker.hpp"

namespace smart_follower_perception
{

class PerceptionFrameProcessor
{
public:
  using PersonPosePublisher =
    rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>;

  PerceptionFrameProcessor(
    const rclcpp::Logger & logger,
    rclcpp::Clock & clock,
    PerceptionParams & params,
    const CameraIntrinsics & intrinsics,
    PerceptionDiagnostics & stats,
    Tracker & tracker,
    LockManager & lock_manager,
    YoloDetector & yolo,
    ReidExtractor & reid);

  void process_detection_result(
    DetectionWorkResult result,
    const PersonPosePublisher::SharedPtr & person_pub,
    const std::function<rclcpp::Time()> & now_fn,
    const std::function<void()> & diagnostics_force_update);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock & clock_;
  PerceptionParams & params_;
  const CameraIntrinsics & intrinsics_;
  PerceptionDiagnostics & stats_;
  Tracker & tracker_;
  LockManager & lock_manager_;
  YoloDetector & yolo_;
  ReidExtractor & reid_;
};

}  // namespace smart_follower_perception
