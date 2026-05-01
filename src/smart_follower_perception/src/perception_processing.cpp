#include "smart_follower_perception/perception_processing.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include <cv_bridge/cv_bridge.h>

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

bool convert_depth_message(
  const Image::SharedPtr & depth_msg,
  cv::Mat & depth_image,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  if (!depth_msg) {
    return false;
  }

  try {
    if (depth_msg->encoding == "16UC1" || depth_msg->encoding == "32FC1") {
      depth_image = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
    } else {
      depth_image = cv_bridge::toCvShare(depth_msg)->image;
    }
    return !depth_image.empty();
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR_THROTTLE(logger, clock, 2000, "depth cv_bridge error: %s", ex.what());
    depth_image.release();
    return false;
  }
}
}  // namespace

PerceptionFrameProcessor::PerceptionFrameProcessor(
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
  intrinsics_(intrinsics),
  stats_(stats),
  tracker_(tracker),
  lock_manager_(lock_manager),
  yolo_(yolo),
  reid_(reid)
{
}

void PerceptionFrameProcessor::process_detection_result(
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
  const std::size_t active_track_count_before = tracker_.tracks().size();
  Tracker::RunResult track_result;
  bool tracking_ok = true;
  try {
    track_result = tracker_.run_tracking(
      std::move(result.detections),
      result.image_size,
      result.stamp,
      result.run_detect);
  } catch (const cv::Exception & ex) {
    tracking_ok = false;
    tracker_.reset();
    RCLCPP_ERROR(
      logger_,
      "[%s] tracker cv exception: %s | stamp_ns=%lld detections=%zu tracks_before=%zu run_detect=%d image=%dx%d lock=(id:%d state:%u). tracker reset and continuing with empty track set.",
      kRuntimeVersion,
      ex.what(),
      static_cast<long long>(result.stamp.nanoseconds()),
      detection_count,
      active_track_count_before,
      result.run_detect ? 1 : 0,
      result.image_size.width,
      result.image_size.height,
      lock_manager_.lock_id(),
      lock_manager_.lock_state());
  } catch (const std::exception & ex) {
    tracking_ok = false;
    tracker_.reset();
    RCLCPP_ERROR(
      logger_,
      "[%s] tracker exception: %s | stamp_ns=%lld detections=%zu tracks_before=%zu run_detect=%d image=%dx%d lock=(id:%d state:%u). tracker reset and continuing with empty track set.",
      kRuntimeVersion,
      ex.what(),
      static_cast<long long>(result.stamp.nanoseconds()),
      detection_count,
      active_track_count_before,
      result.run_detect ? 1 : 0,
      result.image_size.width,
      result.image_size.height,
      lock_manager_.lock_id(),
      lock_manager_.lock_state());
  } catch (...) {
    tracking_ok = false;
    tracker_.reset();
    RCLCPP_ERROR(
      logger_,
      "[%s] tracker threw unknown exception | stamp_ns=%lld detections=%zu tracks_before=%zu run_detect=%d image=%dx%d lock=(id:%d state:%u). tracker reset and continuing with empty track set.",
      kRuntimeVersion,
      static_cast<long long>(result.stamp.nanoseconds()),
      detection_count,
      active_track_count_before,
      result.run_detect ? 1 : 0,
      result.image_size.width,
      result.image_size.height,
      lock_manager_.lock_id(),
      lock_manager_.lock_state());
  }
  const auto tracking_end = Clock::now();
  if (tracking_ok && track_result.recovered_track_id.has_value()) {
    lock_manager_.set_lock_id(*track_result.recovered_track_id);
  }

  const auto lock_begin = Clock::now();
  lock_manager_.update(tracker_.tracks(), result.image_size, result.stamp);
  const auto lock_end = Clock::now();

  cv::Mat depth_image;
  const bool depth_ready = convert_depth_message(result.depth_frame, depth_image, logger_, clock_);
  if (!depth_ready) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      clock_,
      1200,
      "[%s] depth frame unavailable for pose build: lock_id=%d detections=%zu encoding=%s",
      kRuntimeVersion,
      lock_manager_.lock_id(),
      detection_count,
      result.depth_frame ? result.depth_frame->encoding.c_str() : "null");
  }

  const auto message_begin = Clock::now();
  auto pose_build = build_person_pose_array(
    tracker_.tracks(),
    result.color_header,
    depth_image,
    result.image_size,
    lock_manager_.lock_id(),
    lock_manager_.lock_state(),
    intrinsics_,
    params_.camera,
    params_.depth_compare,
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
    RCLCPP_DEBUG_THROTTLE(
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
  stats_.last_depth_samples_valid = pose_build.stats.depth_samples_valid;
  stats_.depth_invalid_count += pose_build.stats.position_failure_count;
  if (pose_build.stats.last_valid_depth_m > 0.0) {
    stats_.last_valid_depth_m = pose_build.stats.last_valid_depth_m;
  }
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

  if (pose_build.stats.position_failure_count > 0) {
    const smart_follower_msgs::msg::TrackedPerson * locked_person = nullptr;
    bool locked_person_invalid = false;
    for (const auto & person : out.persons) {
      if (person.track_id != out.lock_id) {
        continue;
      }
      locked_person = &person;
      locked_person_invalid = !std::isfinite(person.position.x) || !std::isfinite(person.position.y);
      break;
    }

    if (locked_person != nullptr && locked_person_invalid) {
      const bool touch_top = locked_person->bbox.y_offset <= 1U;
      const bool touch_left = locked_person->bbox.x_offset <= 1U;
      const bool touch_right = (locked_person->bbox.x_offset + locked_person->bbox.width + 1U) >= static_cast<unsigned int>(result.image_size.width);
      const bool touch_bottom = (locked_person->bbox.y_offset + locked_person->bbox.height + 1U) >= static_cast<unsigned int>(result.image_size.height);
      const char * reason = "unknown";
      if (!depth_ready) {
        reason = "depth_frame_unavailable";
      } else if (pose_build.stats.depth_samples_valid == 0) {
        reason = "depth_window_no_valid_samples";
      } else if (touch_top || touch_left || touch_right || touch_bottom) {
        reason = "bbox_near_image_edge";
      } else {
        reason = "depth_projection_rejected";
      }
      RCLCPP_WARN_THROTTLE(
        logger_,
        clock_,
        600,
        "[%s] locked target position invalid: reason=%s lock_id=%d state=%u bbox=(%u,%u,%u,%u) touch=(t:%d l:%d r:%d b:%d) img=(%d,%d) depth_ready=%d depth_samples_valid=%zu last_valid_depth_m=%.3f failures=%zu detections=%zu tracks=%zu",
        kRuntimeVersion,
        reason,
        out.lock_id,
        out.lock_state,
        locked_person->bbox.x_offset,
        locked_person->bbox.y_offset,
        locked_person->bbox.width,
        locked_person->bbox.height,
        touch_top ? 1 : 0,
        touch_left ? 1 : 0,
        touch_right ? 1 : 0,
        touch_bottom ? 1 : 0,
        result.image_size.width,
        result.image_size.height,
        depth_ready ? 1 : 0,
        pose_build.stats.depth_samples_valid,
        stats_.last_valid_depth_m,
        pose_build.stats.position_failure_count,
        detection_count,
        out.persons.size());
    } else {
      const char * reason = (out.lock_id < 0) ? "no_lock" : ((locked_person == nullptr) ? "locked_track_missing_from_output" : "non_locked_track_invalid");
      RCLCPP_WARN_THROTTLE(
        logger_,
        clock_,
        1000,
        "[%s] position build produced invalid targets: reason=%s failures=%zu lock_id=%d lock_state=%u depth_samples_valid=%zu last_valid_depth_m=%.3f detections=%zu tracks=%zu",
        kRuntimeVersion,
        reason,
        pose_build.stats.position_failure_count,
        out.lock_id,
        out.lock_state,
        pose_build.stats.depth_samples_valid,
        stats_.last_valid_depth_m,
        detection_count,
        out.persons.size());
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    logger_,
    clock_,
    5000,
    "[%s] profile avg_ms total=%.2f cv_bridge=%.2f yolo=%.2f reid=%.2f recover=%.2f tracking=%.2f lock=%.2f depth_position=%.2f msg_fill=%.2f message=%.2f publish=%.2f | last_ms total=%.2f yolo=%.2f reid=%.2f depth_position=%.2f message=%.2f det=%zu tracks=%zu run_detect=%d",
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
