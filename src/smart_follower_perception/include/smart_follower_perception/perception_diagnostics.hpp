#pragma once

#include <cstddef>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace smart_follower_perception
{

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
  }

  static double stamp_seconds_or_negative(const rclcpp::Time & stamp)
  {
    return stamp.nanoseconds() > 0 ? stamp.seconds() : -1.0;
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
    const rclcpp::Time & now) const
  {
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
    stat.add("lock_id", lock_id);
    stat.add("lock_state", static_cast<int>(lock_state));
    stat.add(
      "lock_age_s",
      last_lock_confirmed_time.nanoseconds() > 0 ? (now - last_lock_confirmed_time).seconds() : -1.0);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Perception healthy");
  }
};

}  // namespace smart_follower_perception
