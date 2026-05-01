#include "smart_follower_perception/perception_pipeline.hpp"

#include <utility>

#include "smart_follower_perception/constants.hpp"

namespace smart_follower_perception
{

PerceptionPipeline::PerceptionPipeline(
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock,
  PerceptionParams & params,
  const CameraIntrinsics & intrinsics,
  PerceptionDiagnostics & stats,
  Tracker & tracker,
  LockManager & lock_manager,
  YoloDetector & yolo,
  ReidExtractor & reid)
: logger_(logger),
  clock_(clock),
  params_(params),
  stats_(stats),
  yolo_(yolo),
  reid_(reid),
  frame_processor_(logger, clock, params, intrinsics, stats, tracker, lock_manager, yolo, reid)
{
}

PerceptionPipeline::~PerceptionPipeline()
{
  stop_detection_worker();
}

bool PerceptionPipeline::enqueue_synchronized_frame(const SynchronizedFrame & synced_frame, bool active)
{
  if (!active || !synced_frame.color || !synced_frame.depth) {
    return false;
  }

  stats_.queued_color_count += 1;
  const bool run_pipeline = (stats_.queued_color_count % params_.process_every_n_frames == 0);
  if (!run_pipeline) {
    stats_.skipped_synced_frame_count += 1;
    RCLCPP_DEBUG_THROTTLE(
      logger_,
      clock_,
      2000,
      "[%s] color/depth callback queued=%zu processed=%d run_pipeline=0",
      kRuntimeVersion,
      stats_.queued_color_count,
      stats_.processed_frame_counter);
    return false;
  }

  scheduled_frame_counter_ += 1;
  const bool run_detect = (scheduled_frame_counter_ % params_.detect_every_n_frames == 0);
  RCLCPP_DEBUG_THROTTLE(
    logger_,
    clock_,
    2000,
    "[%s] color/depth callback queued=%zu scheduled=%d processed=%d run_pipeline=1 run_detect=%d",
    kRuntimeVersion,
    stats_.queued_color_count,
    scheduled_frame_counter_,
    stats_.processed_frame_counter,
    run_detect ? 1 : 0);

  std::lock_guard<std::mutex> worker_lock(worker_mutex_);
  if (!worker_running_) {
    return false;
  }

  DetectionWorkItem item;
  item.frame = synced_frame;
  item.run_detect = run_detect;
  if (pending_work_queue_.size() >= max_pending_work_items_) {
    pending_work_queue_.pop_front();
    stats_.dropped_pending_work_count += 1;
  }
  pending_work_queue_.push_back(std::move(item));
  worker_cv_.notify_one();
  return true;
}

void PerceptionPipeline::start_detection_worker()
{
  std::lock_guard<std::mutex> worker_lock(worker_mutex_);
  if (worker_running_) {
    return;
  }

  worker_running_ = true;
  pending_work_queue_.clear();
  detection_worker_ = std::thread(&PerceptionPipeline::detection_worker_loop, this);
}

void PerceptionPipeline::stop_detection_worker()
{
  {
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    if (!worker_running_ && !detection_worker_.joinable()) {
      pending_work_queue_.clear();
    } else {
      worker_running_ = false;
      pending_work_queue_.clear();
    }
  }
  worker_cv_.notify_all();
  if (detection_worker_.joinable()) {
    detection_worker_.join();
  }
}

void PerceptionPipeline::clear_async_state()
{
  {
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    pending_work_queue_.clear();
  }
  {
    std::lock_guard<std::mutex> result_lock(result_mutex_);
    ready_result_queue_.clear();
  }
  scheduled_frame_counter_ = 0;
}

bool PerceptionPipeline::consume_ready_result(
  const PersonPosePublisher::SharedPtr & person_pub,
  const std::function<rclcpp::Time()> & now_fn,
  const std::function<void()> & diagnostics_force_update)
{
  std::optional<DetectionWorkResult> result;
  {
    std::lock_guard<std::mutex> result_lock(result_mutex_);
    if (ready_result_queue_.empty()) {
      return false;
    }
    result = std::move(ready_result_queue_.front());
    ready_result_queue_.pop_front();
  }

  frame_processor_.process_detection_result(
    std::move(*result), person_pub, now_fn, diagnostics_force_update);
  return true;
}

void PerceptionPipeline::detection_worker_loop()
{
  while (true) {
    DetectionWorkItem item;
    {
      std::unique_lock<std::mutex> worker_lock(worker_mutex_);
      worker_cv_.wait(worker_lock, [this]() {
        return !worker_running_ || !pending_work_queue_.empty();
      });

      if (!worker_running_ && pending_work_queue_.empty()) {
        break;
      }

      item = std::move(pending_work_queue_.front());
      pending_work_queue_.pop_front();
    }

    DetectionWorkResult result;
    const bool ok = run_detection_work_item(
      item,
      yolo_,
      reid_,
      result,
      logger_,
      clock_);
    if (!ok) {
      continue;
    }

    std::lock_guard<std::mutex> result_lock(result_mutex_);
    if (ready_result_queue_.size() >= max_ready_result_items_) {
      ready_result_queue_.pop_front();
      stats_.dropped_ready_result_count += 1;
    }
    ready_result_queue_.push_back(std::move(result));
  }
}

}  // namespace smart_follower_perception

