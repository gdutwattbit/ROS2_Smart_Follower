#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

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
  double last_tf_lookup_ms{0.0};
  double last_tf_transform_ms{0.0};
  double last_message_fill_ms{0.0};
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
  double sum_tf_lookup_ms{0.0};
  double sum_tf_transform_ms{0.0};
  double sum_message_fill_ms{0.0};
  double sum_message_ms{0.0};
  double sum_publish_ms{0.0};
  double sum_total_ms{0.0};

  void reset();
  void observe(
    double camera_info_ms,
    double cv_bridge_ms,
    double yolo_ms,
    double depth_ms,
    double reid_ms,
    double recover_ms,
    double tracking_ms,
    double lock_ms,
    double tf_lookup_ms,
    double tf_transform_ms,
    double message_fill_ms,
    double message_ms,
    double publish_ms,
    double total_ms,
    bool run_detect,
    std::size_t detection_count,
    std::size_t track_count);
  double avg(double sum) const;
  double avg_detections_per_frame() const;
  double avg_tracks_per_frame() const;
  double avg_detect_interval() const;
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

  void reset();
  static double stamp_seconds_or_negative(const rclcpp::Time & stamp);
  static double age_seconds_or_negative(const rclcpp::Time & stamp, const rclcpp::Time & now);
  std::size_t total_dropped_sync_frames(std::size_t sync_dropped) const;
  void fill_status(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    std::size_t active_tracks,
    std::size_t sync_dropped,
    int lock_id,
    uint8_t lock_state,
    const rclcpp::Time & last_lock_confirmed_time,
    bool yolo_ready,
    bool reid_ready,
    const rclcpp::Time & now) const;
};

}  // namespace smart_follower_perception
