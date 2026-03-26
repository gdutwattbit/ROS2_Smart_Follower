#include "smart_follower_perception/perception_diagnostics.hpp"

#include <algorithm>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace smart_follower_perception
{

void PerceptionPipelineProfile::reset()
{
  *this = PerceptionPipelineProfile{};
}

void PerceptionPipelineProfile::observe(
  double cv_bridge_ms,
  double yolo_ms,
  double reid_ms,
  double recover_ms,
  double tracking_ms,
  double lock_ms,
  double position_projection_ms,
  double message_fill_ms,
  double message_ms,
  double publish_ms,
  double total_ms,
  bool run_detect,
  std::size_t detection_count,
  std::size_t track_count)
{
  last_cv_bridge_ms = cv_bridge_ms;
  last_yolo_ms = yolo_ms;
  last_reid_ms = reid_ms;
  last_recover_ms = recover_ms;
  last_tracking_ms = tracking_ms;
  last_lock_ms = lock_ms;
  last_position_projection_ms = position_projection_ms;
  last_message_fill_ms = message_fill_ms;
  last_message_ms = message_ms;
  last_publish_ms = publish_ms;
  last_total_ms = total_ms;
  last_run_detect = run_detect;
  last_detection_count = detection_count;
  last_track_count = track_count;

  sample_count += 1;
  if (run_detect) {
    detect_frame_count += 1;
  }
  total_detections += detection_count;
  total_tracks_published += track_count;

  sum_cv_bridge_ms += cv_bridge_ms;
  sum_yolo_ms += yolo_ms;
  sum_reid_ms += reid_ms;
  sum_recover_ms += recover_ms;
  sum_tracking_ms += tracking_ms;
  sum_lock_ms += lock_ms;
  sum_position_projection_ms += position_projection_ms;
  sum_message_fill_ms += message_fill_ms;
  sum_message_ms += message_ms;
  sum_publish_ms += publish_ms;
  sum_total_ms += total_ms;
}

double PerceptionPipelineProfile::avg(double sum) const
{
  return sample_count > 0 ? sum / static_cast<double>(sample_count) : 0.0;
}

double PerceptionPipelineProfile::avg_detections_per_frame() const
{
  return sample_count > 0 ? static_cast<double>(total_detections) / static_cast<double>(sample_count) : 0.0;
}

double PerceptionPipelineProfile::avg_tracks_per_frame() const
{
  return sample_count > 0 ? static_cast<double>(total_tracks_published) / static_cast<double>(sample_count) : 0.0;
}

double PerceptionPipelineProfile::avg_detect_interval() const
{
  return detect_frame_count > 0 ? static_cast<double>(sample_count) / static_cast<double>(detect_frame_count) : 0.0;
}

void PerceptionDiagnostics::reset()
{
  raw_color_count = 0;
  raw_depth_count = 0;
  queued_color_count = 0;
  skipped_synced_frame_count = 0;
  person_pose_publish_count = 0;
  last_detection_count = 0;
  position_valid_count = 0;
  position_invalid_count = 0;
  last_depth_samples_valid = 0;
  depth_invalid_count = 0;
  intrinsics_ready = false;
  intrinsics_source = "uninitialized";
  depth_source_mode = "depth_compare";
  camera_fx = 0.0;
  camera_fy = 0.0;
  camera_cx = 0.0;
  camera_cy = 0.0;
  last_valid_depth_m = -1.0;
  processed_frame_counter = 0;
  last_infer_ms = 0.0;
  last_color_msg_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_depth_msg_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_person_pose_publish_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  profile.reset();
}

double PerceptionDiagnostics::stamp_seconds_or_negative(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() > 0 ? stamp.seconds() : -1.0;
}

double PerceptionDiagnostics::age_seconds_or_negative(const rclcpp::Time & stamp, const rclcpp::Time & now)
{
  return stamp.nanoseconds() > 0 ? std::max(0.0, (now - stamp).seconds()) : -1.0;
}

void PerceptionDiagnostics::fill_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  std::size_t active_tracks,
  std::size_t sync_dropped,
  int lock_id,
  uint8_t lock_state,
  const rclcpp::Time & last_lock_confirmed_time,
  bool yolo_ready,
  bool reid_ready,
  const rclcpp::Time & now) const
{
  const double color_age = age_seconds_or_negative(last_color_msg_stamp, now);
  const double depth_age = age_seconds_or_negative(last_depth_msg_stamp, now);
  const double publish_age = age_seconds_or_negative(last_person_pose_publish_stamp, now);
  const std::size_t total_positions = position_valid_count + position_invalid_count;
  const double invalid_ratio = total_positions > 0 ?
    static_cast<double>(position_invalid_count) / static_cast<double>(total_positions) : 0.0;
  const bool depth_ready = raw_depth_count > 0 && depth_age >= 0.0 && depth_age <= 2.0;

  stat.add("active_tracks", static_cast<int>(active_tracks));
  stat.add("last_detection_count", static_cast<int>(last_detection_count));
  stat.add("dropped_color_frames", static_cast<int>(sync_dropped));
  stat.add("raw_color_count", static_cast<int>(raw_color_count));
  stat.add("raw_depth_count", static_cast<int>(raw_depth_count));
  stat.add("queued_color_count", static_cast<int>(queued_color_count));
  stat.add("skipped_color_frame_count", static_cast<int>(skipped_synced_frame_count));
  stat.add("processed_frame_count", processed_frame_counter);
  stat.add("person_pose_publish_count", static_cast<int>(person_pose_publish_count));
  stat.add("last_infer_ms", last_infer_ms);
  stat.add("last_color_age_s", color_age);
  stat.add("last_depth_age_s", depth_age);
  stat.add("last_person_pose_publish_age_s", publish_age);
  stat.add("intrinsics_ready", intrinsics_ready);
  stat.add("intrinsics_source", intrinsics_source);
  stat.add("depth_ready", depth_ready);
  stat.add("depth_source_mode", depth_source_mode);
  stat.add("camera_fx", camera_fx);
  stat.add("camera_fy", camera_fy);
  stat.add("camera_cx", camera_cx);
  stat.add("camera_cy", camera_cy);
  stat.add("position_valid_count", static_cast<int>(position_valid_count));
  stat.add("position_invalid_count", static_cast<int>(position_invalid_count));
  stat.add("position_invalid_ratio", invalid_ratio);
  stat.add("depth_samples_valid", static_cast<int>(last_depth_samples_valid));
  stat.add("depth_invalid_count", static_cast<int>(depth_invalid_count));
  stat.add("last_valid_depth_m", last_valid_depth_m);
  stat.add("depth_position_ms", profile.last_position_projection_ms);
  stat.add("position_projection_ms", profile.last_position_projection_ms);
  stat.add("profile_samples", static_cast<int>(profile.sample_count));
  stat.add("profile_detect_frames", static_cast<int>(profile.detect_frame_count));
  stat.add("profile_last_run_detect", profile.last_run_detect);
  stat.add("profile_last_detection_count", static_cast<int>(profile.last_detection_count));
  stat.add("profile_last_track_count", static_cast<int>(profile.last_track_count));
  stat.add("profile_avg_detections_per_frame", profile.avg_detections_per_frame());
  stat.add("profile_avg_tracks_per_frame", profile.avg_tracks_per_frame());
  stat.add("profile_avg_detect_interval_frames", profile.avg_detect_interval());
  stat.add("profile_last_cv_bridge_ms", profile.last_cv_bridge_ms);
  stat.add("profile_last_yolo_ms", profile.last_yolo_ms);
  stat.add("profile_last_reid_ms", profile.last_reid_ms);
  stat.add("profile_last_recover_ms", profile.last_recover_ms);
  stat.add("profile_last_tracking_ms", profile.last_tracking_ms);
  stat.add("profile_last_lock_ms", profile.last_lock_ms);
  stat.add("profile_last_position_projection_ms", profile.last_position_projection_ms);
  stat.add("profile_last_message_fill_ms", profile.last_message_fill_ms);
  stat.add("profile_last_message_ms", profile.last_message_ms);
  stat.add("profile_last_publish_ms", profile.last_publish_ms);
  stat.add("profile_last_total_ms", profile.last_total_ms);
  stat.add("profile_avg_cv_bridge_ms", profile.avg(profile.sum_cv_bridge_ms));
  stat.add("profile_avg_yolo_ms", profile.avg(profile.sum_yolo_ms));
  stat.add("profile_avg_reid_ms", profile.avg(profile.sum_reid_ms));
  stat.add("profile_avg_recover_ms", profile.avg(profile.sum_recover_ms));
  stat.add("profile_avg_tracking_ms", profile.avg(profile.sum_tracking_ms));
  stat.add("profile_avg_lock_ms", profile.avg(profile.sum_lock_ms));
  stat.add("profile_avg_depth_position_ms", profile.avg(profile.sum_position_projection_ms));
  stat.add("profile_avg_position_projection_ms", profile.avg(profile.sum_position_projection_ms));
  stat.add("profile_avg_message_fill_ms", profile.avg(profile.sum_message_fill_ms));
  stat.add("profile_avg_message_ms", profile.avg(profile.sum_message_ms));
  stat.add("profile_avg_publish_ms", profile.avg(profile.sum_publish_ms));
  stat.add("profile_avg_total_ms", profile.avg(profile.sum_total_ms));
  stat.add("yolo_ready", yolo_ready);
  stat.add("reid_ready", reid_ready);
  stat.add("lock_id", lock_id);
  stat.add("lock_state", static_cast<int>(lock_state));
  stat.add(
    "lock_age_s",
    last_lock_confirmed_time.nanoseconds() > 0 ? (now - last_lock_confirmed_time).seconds() : -1.0);

  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Perception healthy";
  const auto raise = [&](int new_level, const std::string & new_message) {
    if (new_level > level) {
      level = new_level;
      message = new_message;
    }
  };

  if (!yolo_ready) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "YOLO runtime not ready");
  } else if (!reid_ready) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "ReID runtime not ready");
  }

  if (!intrinsics_ready) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for camera intrinsics");
  }

  if (raw_color_count == 0 || color_age < 0.0) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for color input");
  } else if (color_age > 2.0) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Color input stale");
  } else if (color_age > 0.5) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Color input delayed");
  }

  if (raw_depth_count == 0 || depth_age < 0.0) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for depth input");
  } else if (depth_age > 2.0) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Depth input stale");
  } else if (depth_age > 0.5) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Depth input delayed");
  }

  if (person_pose_publish_count == 0) {
    if (queued_color_count > 0) {
      raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for first person_pose publish");
    }
  } else if (publish_age > 2.0) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "person_pose publish stalled");
  } else if (publish_age > 0.5) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "person_pose publish delayed");
  }

  if (total_positions >= 10 && invalid_ratio > 0.8) {
    raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Depth positioning failing frequently");
  }

  stat.summary(level, message);
}

}  // namespace smart_follower_perception
