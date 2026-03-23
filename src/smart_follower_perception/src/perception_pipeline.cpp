#include "smart_follower_perception/perception_pipeline.hpp"

#include <chrono>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>

#include "smart_follower_perception/constants.hpp"

namespace smart_follower_perception
{

namespace
{
using Clock = std::chrono::steady_clock;

double elapsed_ms(const Clock::time_point & begin, const Clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

}  // namespace

PerceptionPipeline::PerceptionPipeline(
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock,
  PerceptionParams & params,
  const MonocularCameraIntrinsics & intrinsics,
  PerceptionDiagnostics & stats,
  Tracker & tracker,
  LockManager & lock_manager,
  YoloDetector & yolo,
  ReidExtractor & reid)
: logger_(logger),
  clock_(clock),
  params_(params),
  intrinsics_(intrinsics),
  stats_(stats),
  tracker_(tracker),
  lock_manager_(lock_manager),
  yolo_(yolo),
  reid_(reid)
{
}

PerceptionPipeline::~PerceptionPipeline()
{
  stop_detection_worker();
}

bool PerceptionPipeline::enqueue_synchronized_frame(const SynchronizedFrame & synced_frame, bool active)
{
  if (!active || !synced_frame.color) {
    return false;
  }

  stats_.queued_color_count += 1;
  const bool run_pipeline = (stats_.queued_color_count % params_.process_every_n_frames == 0);
  if (!run_pipeline) {
    stats_.skipped_synced_frame_count += 1;
    RCLCPP_INFO_THROTTLE(
      logger_,
      clock_,
      2000,
      "[%s] color callback queued=%zu processed=%d run_pipeline=0",
      kRuntimeVersion,
      stats_.queued_color_count,
      stats_.processed_frame_counter);
    return false;
  }

  scheduled_frame_counter_ += 1;
  const bool run_detect = (scheduled_frame_counter_ % params_.detect_every_n_frames == 0);
  RCLCPP_INFO_THROTTLE(
    logger_,
    clock_,
    2000,
    "[%s] color callback queued=%zu scheduled=%d processed=%d run_pipeline=1 run_detect=%d",
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
  pending_work_ = std::move(item);
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
  pending_work_.reset();
  detection_worker_ = std::thread(&PerceptionPipeline::detection_worker_loop, this);
}

void PerceptionPipeline::stop_detection_worker()
{
  {
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    if (!worker_running_ && !detection_worker_.joinable()) {
      pending_work_.reset();
    } else {
      worker_running_ = false;
      pending_work_.reset();
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
    pending_work_.reset();
  }
  {
    std::lock_guard<std::mutex> result_lock(result_mutex_);
    latest_result_.reset();
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
    if (!latest_result_.has_value()) {
      return false;
    }
    result = std::move(latest_result_);
    latest_result_.reset();
  }

  process_detection_result(std::move(*result), person_pub, now_fn, diagnostics_force_update);
  return true;
}

void PerceptionPipeline::detection_worker_loop()
{
  while (true) {
    DetectionWorkItem item;
    {
      std::unique_lock<std::mutex> worker_lock(worker_mutex_);
      worker_cv_.wait(worker_lock, [this]() {
        return !worker_running_ || pending_work_.has_value();
      });

      if (!worker_running_ && !pending_work_.has_value()) {
        break;
      }

      item = std::move(*pending_work_);
      pending_work_.reset();
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
    latest_result_ = std::move(result);
  }
}

void PerceptionPipeline::process_detection_result(
  DetectionWorkResult result,
  const PersonPosePublisher::SharedPtr & person_pub,
  const std::function<rclcpp::Time()> & now_fn,
  const std::function<void()> & diagnostics_force_update)
{
  stats_.processed_frame_counter += 1;

  if (!result.reid_dim_error.empty()) {
    RCLCPP_ERROR_THROTTLE(logger_, clock_, 2000, "%s", result.reid_dim_error.c_str());
  }

  double recover_ms = 0.0;
  for (auto & detection : result.detections) {
    if (
      lock_manager_.lock_state() == smart_follower_msgs::msg::PersonPoseArray::LOST &&
      detection.feature_valid)
    {
      const auto recover_begin = Clock::now();
      auto recovered = tracker_.try_recover_lock_from_memory(
        detection,
        result.stamp,
        lock_manager_.lock_id());
      const auto recover_end = Clock::now();
      recover_ms += elapsed_ms(recover_begin, recover_end);
      if (recovered.has_value()) {
        detection.recovered_track_id = *recovered;
      }
    }
  }

  const std::size_t detection_count = result.detections.size();

  const auto tracking_begin = Clock::now();
  auto track_result = tracker_.run_tracking(
    std::move(result.detections),
    result.image_size,
    result.stamp,
    result.run_detect);
  const auto tracking_end = Clock::now();
  if (track_result.recovered_track_id.has_value()) {
    lock_manager_.set_lock_id(*track_result.recovered_track_id);
  }

  const auto lock_begin = Clock::now();
  lock_manager_.update(tracker_.tracks(), result.image_size, result.stamp);
  const auto lock_end = Clock::now();

  const auto message_begin = Clock::now();
  auto pose_build = build_person_pose_array(
    tracker_.tracks(),
    result.color_header,
    result.image_size,
    lock_manager_.lock_id(),
    lock_manager_.lock_state(),
    intrinsics_,
    params_.monocular,
    params_.base_frame);
  const auto message_end = Clock::now();
  auto & out = pose_build.msg;

  double publish_ms = 0.0;
  if (person_pub && person_pub->is_activated()) {
    const auto publish_begin = Clock::now();
    person_pub->publish(out);
    const auto publish_end = Clock::now();
    publish_ms = elapsed_ms(publish_begin, publish_end);
    stats_.person_pose_publish_count += 1;
    stats_.last_person_pose_publish_stamp = now_fn();
    RCLCPP_INFO_THROTTLE(
      logger_,
      clock_,
      2000,
      "[%s] published person_pose publish_count=%zu persons=%zu lock_id=%d lock_state=%u detections=%zu",
      kRuntimeVersion,
      stats_.person_pose_publish_count,
      out.persons.size(),
      out.lock_id,
      out.lock_state,
      detection_count);
  }

  const auto frame_end = Clock::now();
  const double tracking_ms = elapsed_ms(tracking_begin, tracking_end);
  const double lock_ms = elapsed_ms(lock_begin, lock_end);
  const double message_ms = elapsed_ms(message_begin, message_end);
  const double total_ms = elapsed_ms(result.processing_begin, frame_end);

  stats_.last_detection_count = detection_count;
  stats_.last_infer_ms = total_ms;
  stats_.position_valid_count += pose_build.stats.position_success_count;
  stats_.position_invalid_count += pose_build.stats.position_failure_count;
  stats_.profile.observe(
    result.cv_bridge_ms,
    result.yolo_ms,
    result.reid_ms,
    recover_ms,
    tracking_ms,
    lock_ms,
    pose_build.stats.position_projection_ms,
    pose_build.stats.message_fill_ms,
    message_ms,
    publish_ms,
    total_ms,
    result.run_detect,
    detection_count,
    out.persons.size());

  RCLCPP_INFO_THROTTLE(
    logger_,
    clock_,
    5000,
    "[%s] profile avg_ms total=%.2f cv_bridge=%.2f yolo=%.2f reid=%.2f recover=%.2f tracking=%.2f lock=%.2f projection=%.2f msg_fill=%.2f message=%.2f publish=%.2f | last_ms total=%.2f yolo=%.2f reid=%.2f projection=%.2f message=%.2f det=%zu tracks=%zu run_detect=%d",
    kRuntimeVersion,
    stats_.profile.avg(stats_.profile.sum_total_ms),
    stats_.profile.avg(stats_.profile.sum_cv_bridge_ms),
    stats_.profile.avg(stats_.profile.sum_yolo_ms),
    stats_.profile.avg(stats_.profile.sum_reid_ms),
    stats_.profile.avg(stats_.profile.sum_recover_ms),
    stats_.profile.avg(stats_.profile.sum_tracking_ms),
    stats_.profile.avg(stats_.profile.sum_lock_ms),
    stats_.profile.avg(stats_.profile.sum_position_projection_ms),
    stats_.profile.avg(stats_.profile.sum_message_fill_ms),
    stats_.profile.avg(stats_.profile.sum_message_ms),
    stats_.profile.avg(stats_.profile.sum_publish_ms),
    stats_.profile.last_total_ms,
    stats_.profile.last_yolo_ms,
    stats_.profile.last_reid_ms,
    stats_.profile.last_position_projection_ms,
    stats_.profile.last_message_ms,
    detection_count,
    out.persons.size(),
    result.run_detect ? 1 : 0);

  diagnostics_force_update();
}

}  // namespace smart_follower_perception
