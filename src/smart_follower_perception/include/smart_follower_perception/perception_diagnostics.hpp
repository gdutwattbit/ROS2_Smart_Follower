#pragma once

#include <algorithm>
#include <cstddef>
#include <string>

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
  rclcpp::Time last_person_pose_publish_stamp{0, 0, RCL_ROS_TIME};

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
