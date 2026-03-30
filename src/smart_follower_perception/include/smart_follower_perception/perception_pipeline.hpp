#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

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

class PerceptionPipeline
{
public:
  using PersonPosePublisher =
    rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>;

  PerceptionPipeline(
    const rclcpp::Logger & logger,
    rclcpp::Clock & clock,
    PerceptionParams & params,
    const CameraIntrinsics & intrinsics,
    PerceptionDiagnostics & stats,
    Tracker & tracker,
    LockManager & lock_manager,
    YoloDetector & yolo,
    ReidExtractor & reid);

  ~PerceptionPipeline();

  bool enqueue_synchronized_frame(const SynchronizedFrame & synced_frame, bool active);
  void start_detection_worker();
  void stop_detection_worker();
  void clear_async_state();
  bool consume_ready_result(
    const PersonPosePublisher::SharedPtr & person_pub,
    const std::function<rclcpp::Time()> & now_fn,
    const std::function<void()> & diagnostics_force_update);

private:
  void detection_worker_loop();
  void process_detection_result(
    DetectionWorkResult result,
    const PersonPosePublisher::SharedPtr & person_pub,
    const std::function<rclcpp::Time()> & now_fn,
    const std::function<void()> & diagnostics_force_update);

  rclcpp::Logger logger_;
  rclcpp::Clock & clock_;
  PerceptionParams & params_;
  const CameraIntrinsics & intrinsics_;
  PerceptionDiagnostics & stats_;
  Tracker & tracker_;
  LockManager & lock_manager_;
  YoloDetector & yolo_;
  ReidExtractor & reid_;

  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::thread detection_worker_;
  bool worker_running_{false};
  std::deque<DetectionWorkItem> pending_work_queue_;
  std::size_t max_pending_work_items_{4};

  std::mutex result_mutex_;
  std::deque<DetectionWorkResult> ready_result_queue_;
  std::size_t max_ready_result_items_{8};
  int scheduled_frame_counter_{0};
};

}  // namespace smart_follower_perception

