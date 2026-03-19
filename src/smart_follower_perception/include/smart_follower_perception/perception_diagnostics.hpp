#pragma once

#include <algorithm>
#include <cstddef>
#include <string>

#include "smart_follower_perception/constants.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace smart_follower_perception
{

struct PerceptionPipelineProfile
{
  double last_camera_info_ms{0.0};
  double last_cv_bridge_ms{0.0};
  double last_yolo_ms{0.0};
  double last_depth_ms{0.0};
  double last_reid_ms{0.0};
  double last_recover_ms{0.0};
  double last_tracking_ms{0.0};
  double last_lock_ms{0.0};
  double last_message_ms{0.0};
  double last_publish_ms{0.0};
  double last_total_ms{0.0};
  std::size_t sample_count{0};
  std::size_t detect_frame_count{0};
  std::size_t total_detections{0};
  std::size_t total_tracks_published{0};
  bool last_run_detect{false};
  std::size_t last_detection_count{0};
  std::size_t last_track_count{0};

  double sum_camera_info_ms{0.0};
  double sum_cv_bridge_ms{0.0};
  double sum_yolo_ms{0.0};
  double sum_depth_ms{0.0};
  double sum_reid_ms{0.0};
  double sum_recover_ms{0.0};
  double sum_tracking_ms{0.0};
  double sum_lock_ms{0.0};
  double sum_message_ms{0.0};
  double sum_publish_ms{0.0};
  double sum_total_ms{0.0};

  void reset()
  {
    *this = PerceptionPipelineProfile{};
  }

  void observe(
    double camera_info_ms,
    double cv_bridge_ms,
    double yolo_ms,
    double depth_ms,
    double reid_ms,
    double recover_ms,
    double tracking_ms,
    double lock_ms,
    double message_ms,
    double publish_ms,
    double total_ms,
    bool run_detect,
    std::size_t detection_count,
    std::size_t track_count)
  {
    last_camera_info_ms = camera_info_ms;
    last_cv_bridge_ms = cv_bridge_ms;
    last_yolo_ms = yolo_ms;
    last_depth_ms = depth_ms;
    last_reid_ms = reid_ms;
    last_recover_ms = recover_ms;
    last_tracking_ms = tracking_ms;
    last_lock_ms = lock_ms;
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

    sum_camera_info_ms += camera_info_ms;
    sum_cv_bridge_ms += cv_bridge_ms;
    sum_yolo_ms += yolo_ms;
    sum_depth_ms += depth_ms;
    sum_reid_ms += reid_ms;
    sum_recover_ms += recover_ms;
    sum_tracking_ms += tracking_ms;
    sum_lock_ms += lock_ms;
    sum_message_ms += message_ms;
    sum_publish_ms += publish_ms;
    sum_total_ms += total_ms;
  }

  double avg(double sum) const
  {
    return sample_count > 0 ? sum / static_cast<double>(sample_count) : 0.0;
  }

  double avg_detections_per_frame() const
  {
    return sample_count > 0 ? static_cast<double>(total_detections) / static_cast<double>(sample_count) : 0.0;
  }

  double avg_tracks_per_frame() const
  {
    return sample_count > 0 ? static_cast<double>(total_tracks_published) / static_cast<double>(sample_count) : 0.0;
  }

  double avg_detect_interval() const
  {
    return detect_frame_count > 0 ? static_cast<double>(sample_count) / static_cast<double>(detect_frame_count) : 0.0;
  }
};

struct PerceptionDiagnostics
{
  std::size_t raw_color_count{0};
  std::size_t raw_depth_count{0};
  std::size_t raw_info_count{0};
  std::size_t synced_callback_count{0};
  std::size_t skipped_synced_frame_count{0};
  std::size_t person_pose_publish_count{0};
  std::size_t last_detection_count{0};
  std::size_t dropped_sync_frames_extra{0};
  int synced_frame_counter{0};
  int processed_frame_counter{0};
  double last_infer_ms{0.0};
  rclcpp::Time last_color_msg_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_msg_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_info_msg_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_person_pose_publish_stamp{0, 0, RCL_ROS_TIME};
  PerceptionPipelineProfile profile{};

  void reset()
  {
    raw_color_count = 0;
    raw_depth_count = 0;
    raw_info_count = 0;
    synced_callback_count = 0;
    skipped_synced_frame_count = 0;
    person_pose_publish_count = 0;
    last_detection_count = 0;
    dropped_sync_frames_extra = 0;
    synced_frame_counter = 0;
    processed_frame_counter = 0;
    last_infer_ms = 0.0;
    last_color_msg_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_depth_msg_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_info_msg_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_person_pose_publish_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    profile.reset();
  }

  static double stamp_seconds_or_negative(const rclcpp::Time & stamp)
  {
    return stamp.nanoseconds() > 0 ? stamp.seconds() : -1.0;
  }

  static double age_seconds_or_negative(const rclcpp::Time & stamp, const rclcpp::Time & now)
  {
    return stamp.nanoseconds() > 0 ? std::max(0.0, (now - stamp).seconds()) : -1.0;
  }

  std::size_t total_dropped_sync_frames(std::size_t sync_dropped) const
  {
    return sync_dropped + dropped_sync_frames_extra;
  }

  void fill_status(
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
    const double info_age = age_seconds_or_negative(last_info_msg_stamp, now);
    const double publish_age = age_seconds_or_negative(last_person_pose_publish_stamp, now);

    stat.add("active_tracks", static_cast<int>(active_tracks));
    stat.add("last_detection_count", static_cast<int>(last_detection_count));
    stat.add("dropped_sync_frames", static_cast<int>(total_dropped_sync_frames(sync_dropped)));
    stat.add("raw_color_count", static_cast<int>(raw_color_count));
    stat.add("raw_depth_count", static_cast<int>(raw_depth_count));
    stat.add("raw_info_count", static_cast<int>(raw_info_count));
    stat.add("synced_callback_count", static_cast<int>(synced_callback_count));
    stat.add("skipped_synced_frame_count", static_cast<int>(skipped_synced_frame_count));
    stat.add("processed_frame_count", processed_frame_counter);
    stat.add("person_pose_publish_count", static_cast<int>(person_pose_publish_count));
    stat.add("last_infer_ms", last_infer_ms);
    stat.add("last_color_age_s", color_age);
    stat.add("last_depth_age_s", depth_age);
    stat.add("last_info_age_s", info_age);
    stat.add("last_person_pose_publish_age_s", publish_age);
    stat.add("profile_samples", static_cast<int>(profile.sample_count));
    stat.add("profile_detect_frames", static_cast<int>(profile.detect_frame_count));
    stat.add("profile_last_run_detect", profile.last_run_detect);
    stat.add("profile_last_detection_count", static_cast<int>(profile.last_detection_count));
    stat.add("profile_last_track_count", static_cast<int>(profile.last_track_count));
    stat.add("profile_avg_detections_per_frame", profile.avg_detections_per_frame());
    stat.add("profile_avg_tracks_per_frame", profile.avg_tracks_per_frame());
    stat.add("profile_avg_detect_interval_frames", profile.avg_detect_interval());
    stat.add("profile_last_camera_info_ms", profile.last_camera_info_ms);
    stat.add("profile_last_cv_bridge_ms", profile.last_cv_bridge_ms);
    stat.add("profile_last_yolo_ms", profile.last_yolo_ms);
    stat.add("profile_last_depth_ms", profile.last_depth_ms);
    stat.add("profile_last_reid_ms", profile.last_reid_ms);
    stat.add("profile_last_recover_ms", profile.last_recover_ms);
    stat.add("profile_last_tracking_ms", profile.last_tracking_ms);
    stat.add("profile_last_lock_ms", profile.last_lock_ms);
    stat.add("profile_last_message_ms", profile.last_message_ms);
    stat.add("profile_last_publish_ms", profile.last_publish_ms);
    stat.add("profile_last_total_ms", profile.last_total_ms);
    stat.add("profile_avg_camera_info_ms", profile.avg(profile.sum_camera_info_ms));
    stat.add("profile_avg_cv_bridge_ms", profile.avg(profile.sum_cv_bridge_ms));
    stat.add("profile_avg_yolo_ms", profile.avg(profile.sum_yolo_ms));
    stat.add("profile_avg_depth_ms", profile.avg(profile.sum_depth_ms));
    stat.add("profile_avg_reid_ms", profile.avg(profile.sum_reid_ms));
    stat.add("profile_avg_recover_ms", profile.avg(profile.sum_recover_ms));
    stat.add("profile_avg_tracking_ms", profile.avg(profile.sum_tracking_ms));
    stat.add("profile_avg_lock_ms", profile.avg(profile.sum_lock_ms));
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

    const auto assess_input = [&](const char * label, std::size_t count, double age_s) {
      if (count == 0 || age_s < 0.0) {
        raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string("Waiting for ") + label + " input");
        return;
      }
      if (age_s > 2.0) {
        raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string(label) + " input stale");
      } else if (age_s > 0.5) {
        raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string(label) + " input delayed");
      }
    };

    assess_input("color", raw_color_count, color_age);
    assess_input("depth", raw_depth_count, depth_age);
    assess_input("camera_info", raw_info_count, info_age);

    if (person_pose_publish_count == 0) {
      if (synced_callback_count > 0) {
        raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for first person_pose publish");
      }
    } else if (publish_age > 2.0) {
      raise(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "person_pose publish stalled");
    } else if (publish_age > 0.5) {
      raise(diagnostic_msgs::msg::DiagnosticStatus::WARN, "person_pose publish delayed");
    }

    stat.summary(level, message);
  }
};

}  // namespace smart_follower_perception
