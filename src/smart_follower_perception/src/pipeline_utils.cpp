#include "smart_follower_perception/pipeline_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <cv_bridge/cv_bridge.h>

#include "smart_follower_perception/geometry_utils.hpp"

namespace smart_follower_perception
{

bool is_sync_pair_within_slop(
  const Image::SharedPtr & color_msg,
  const Image::SharedPtr & depth_msg,
  double sync_slop_sec)
{
  if (!color_msg || !depth_msg) {
    return false;
  }

  const rclcpp::Time color_stamp(color_msg->header.stamp);
  const rclcpp::Time depth_stamp(depth_msg->header.stamp);
  return std::abs((color_stamp - depth_stamp).seconds()) <= sync_slop_sec;
}

bool run_detection_work_item(
  const DetectionWorkItem & item,
  YoloDetector & yolo,
  ReidExtractor & reid,
  float depth_min_m,
  float depth_max_m,
  DetectionWorkResult & result,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  if (!item.frame.color || !item.frame.depth || !item.frame.info) {
    return false;
  }

  using SteadyClock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const SteadyClock::time_point & begin, const SteadyClock::time_point & end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };

  result = DetectionWorkResult();
  result.processing_begin = SteadyClock::now();
  result.stamp = rclcpp::Time(item.frame.color->header.stamp);
  result.color_header = item.frame.color->header;
  result.depth_header = item.frame.depth->header;
  result.camera_info = item.frame.info;
  result.run_detect = item.run_detect;

  cv::Mat color;
  cv::Mat depth;
  try {
    const auto cv_begin = SteadyClock::now();
    color = cv_bridge::toCvShare(item.frame.color, "bgr8")->image;
    if (item.frame.depth->encoding == "16UC1" || item.frame.depth->encoding == "32FC1") {
      depth = cv_bridge::toCvShare(item.frame.depth, item.frame.depth->encoding)->image;
    } else {
      depth = cv_bridge::toCvShare(item.frame.depth)->image;
    }
    const auto cv_end = SteadyClock::now();
    result.cv_bridge_ms = elapsed_ms(cv_begin, cv_end);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR_THROTTLE(logger, clock, 2000, "cv_bridge error: %s", ex.what());
    return false;
  }

  result.image_size = color.size();

  if (item.run_detect) {
    const auto yolo_begin = SteadyClock::now();
    auto detector_results = yolo.detect(color);
    const auto yolo_end = SteadyClock::now();
    result.yolo_ms = elapsed_ms(yolo_begin, yolo_end);

    result.detections.reserve(detector_results.size());
    for (const auto & det : detector_results) {
      Detection detection;
      detection.bbox = det.bbox;
      detection.confidence = det.conf;
      const int cx = static_cast<int>(detection.bbox.x + detection.bbox.width * 0.5F);
      const int cy = static_cast<int>(detection.bbox.y + detection.bbox.height * 0.5F);

      const auto depth_begin = SteadyClock::now();
      detection.depth_m = sample_depth_m(depth, cx, cy, depth_min_m, depth_max_m);
      const auto depth_end = SteadyClock::now();
      result.depth_ms += elapsed_ms(depth_begin, depth_end);

      bool feature_valid = false;
      const auto reid_begin = SteadyClock::now();
      detection.feature = reid.extract(color, detection.bbox, feature_valid);
      const auto reid_end = SteadyClock::now();
      result.reid_ms += elapsed_ms(reid_begin, reid_end);
      detection.feature_valid = feature_valid;

      result.detections.push_back(detection);
    }
  }

  reid.consume_output_dim_error(result.reid_dim_error);
  return true;
}

smart_follower_msgs::msg::TrackedPerson track_to_message(
  const Track & track,
  image_geometry::PinholeCameraModel & camera_model,
  const geometry_msgs::msg::TransformStamped * camera_to_base_tf,
  float depth_min_m,
  float depth_max_m,
  MessageBuildStats * stats)
{
  using SteadyClock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const SteadyClock::time_point & begin, const SteadyClock::time_point & end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };

  smart_follower_msgs::msg::TrackedPerson msg;
  msg.track_id = track.id;
  msg.track_state = track.state;
  msg.confidence = track.confidence;
  msg.bbox.x_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.x));
  msg.bbox.y_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.y));
  msg.bbox.width = static_cast<uint32_t>(std::max(0.0F, track.bbox.width));
  msg.bbox.height = static_cast<uint32_t>(std::max(0.0F, track.bbox.height));

  if (camera_to_base_tf != nullptr) {
    const auto tf_begin = SteadyClock::now();
    auto position = pixel_to_base_point(
      track.bbox,
      track.depth_m,
      camera_model,
      *camera_to_base_tf,
      depth_min_m,
      depth_max_m);
    const auto tf_end = SteadyClock::now();
    if (stats != nullptr) {
      stats->tf_transform_ms += elapsed_ms(tf_begin, tf_end);
    }

    if (position.has_value()) {
      msg.position = *position;
      if (stats != nullptr) {
        stats->tf_success_count += 1;
      }
    } else {
      msg.position.x = std::numeric_limits<double>::quiet_NaN();
      msg.position.y = std::numeric_limits<double>::quiet_NaN();
      msg.position.z = std::numeric_limits<double>::quiet_NaN();
      if (stats != nullptr) {
        stats->tf_failure_count += 1;
      }
    }
  } else {
    msg.position.x = std::numeric_limits<double>::quiet_NaN();
    msg.position.y = std::numeric_limits<double>::quiet_NaN();
    msg.position.z = std::numeric_limits<double>::quiet_NaN();
    if (stats != nullptr) {
      stats->tf_failure_count += 1;
    }
  }

  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.depth_m = track.depth_m;
  if (track.feature_valid) {
    std::copy(track.ema_feature.begin(), track.ema_feature.end(), msg.appearance_feature.begin());
  }
  msg.last_seen = track.last_seen;
  return msg;
}

PersonPoseBuildResult build_person_pose_array(
  const std::unordered_map<int, Track> & tracks,
  const std_msgs::msg::Header & color_header,
  const std_msgs::msg::Header & depth_header,
  int lock_id,
  uint8_t lock_state,
  image_geometry::PinholeCameraModel & camera_model,
  tf2_ros::Buffer & tf_buffer,
  const std::string & base_frame,
  float depth_min_m,
  float depth_max_m,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  using SteadyClock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const SteadyClock::time_point & begin, const SteadyClock::time_point & end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };

  PersonPoseBuildResult result;
  auto & out = result.msg;
  out.header = color_header;
  out.header.frame_id = base_frame;
  out.lock_id = lock_id;
  out.lock_state = lock_state;
  out.persons.reserve(tracks.size());

  std::optional<geometry_msgs::msg::TransformStamped> camera_to_base_tf;
  if (!tracks.empty()) {
    const auto tf_lookup_begin = SteadyClock::now();
    camera_to_base_tf = lookup_camera_to_base_transform(
      depth_header,
      tf_buffer,
      base_frame,
      logger,
      clock);
    const auto tf_lookup_end = SteadyClock::now();
    result.stats.tf_lookup_ms = elapsed_ms(tf_lookup_begin, tf_lookup_end);
  }

  const auto message_fill_begin = SteadyClock::now();
  for (const auto & kv : tracks) {
    out.persons.push_back(track_to_message(
      kv.second,
      camera_model,
      camera_to_base_tf ? &(*camera_to_base_tf) : nullptr,
      depth_min_m,
      depth_max_m,
      &result.stats));
  }
  const auto message_fill_end = SteadyClock::now();
  result.stats.message_fill_ms = elapsed_ms(message_fill_begin, message_fill_end);
  return result;
}

}  // namespace smart_follower_perception
