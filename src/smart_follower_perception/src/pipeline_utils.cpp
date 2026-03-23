#include "smart_follower_perception/pipeline_utils.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

#include <cv_bridge/cv_bridge.h>

namespace smart_follower_perception
{

bool run_detection_work_item(
  const DetectionWorkItem & item,
  YoloDetector & yolo,
  ReidExtractor & reid,
  DetectionWorkResult & result,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  if (!item.frame.color) {
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
  result.run_detect = item.run_detect;

  cv::Mat color;
  try {
    const auto cv_begin = SteadyClock::now();
    color = cv_bridge::toCvShare(item.frame.color, "bgr8")->image;
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
    result.yolo_preprocess_ms = yolo.last_profile().preprocess_ms;
    result.yolo_run_ms = yolo.last_profile().run_ms;
    result.yolo_postprocess_ms = yolo.last_profile().postprocess_ms;

    result.detections.reserve(detector_results.size());
    for (const auto & det : detector_results) {
      Detection detection;
      detection.bbox = det.bbox;
      detection.confidence = det.conf;

      bool feature_valid = false;
      const auto reid_begin = SteadyClock::now();
      detection.feature = reid.extract(color, detection.bbox, feature_valid);
      const auto reid_end = SteadyClock::now();
      result.reid_ms += elapsed_ms(reid_begin, reid_end);
      result.reid_preprocess_ms += reid.last_profile().preprocess_ms;
      result.reid_run_ms += reid.last_profile().run_ms;
      detection.feature_valid = feature_valid;

      result.detections.push_back(detection);
    }
  }

  reid.consume_output_dim_error(result.reid_dim_error);
  return true;
}

smart_follower_msgs::msg::TrackedPerson track_to_message(
  const Track & track,
  const cv::Size & image_size,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  MessageBuildStats * stats)
{
  using SteadyClock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const SteadyClock::time_point & begin, const SteadyClock::time_point & end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };

  (void)image_size;

  smart_follower_msgs::msg::TrackedPerson msg;
  msg.track_id = track.id;
  msg.track_state = track.state;
  msg.confidence = track.confidence;
  msg.bbox.x_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.x));
  msg.bbox.y_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.y));
  msg.bbox.width = static_cast<uint32_t>(std::max(0.0F, track.bbox.width));
  msg.bbox.height = static_cast<uint32_t>(std::max(0.0F, track.bbox.height));

  const auto position_begin = SteadyClock::now();
  auto position = estimate_person_position_from_bbox(track.bbox, intrinsics, monocular);
  const auto position_end = SteadyClock::now();
  if (stats != nullptr) {
    stats->position_projection_ms += elapsed_ms(position_begin, position_end);
  }

  if (position.has_value()) {
    msg.position = *position;
    if (stats != nullptr) {
      stats->position_success_count += 1;
    }
  } else {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    msg.position.x = nan;
    msg.position.y = nan;
    msg.position.z = nan;
    if (stats != nullptr) {
      stats->position_failure_count += 1;
    }
  }

  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.depth_m = std::numeric_limits<float>::quiet_NaN();
  if (track.feature_valid) {
    std::copy(track.ema_feature.begin(), track.ema_feature.end(), msg.appearance_feature.begin());
  }
  msg.last_seen = track.last_seen;
  return msg;
}

PersonPoseBuildResult build_person_pose_array(
  const std::unordered_map<int, Track> & tracks,
  const std_msgs::msg::Header & color_header,
  const cv::Size & image_size,
  int lock_id,
  uint8_t lock_state,
  const MonocularCameraIntrinsics & intrinsics,
  const MonocularPositionConfig & monocular,
  const std::string & base_frame)
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

  const auto message_fill_begin = SteadyClock::now();
  for (const auto & kv : tracks) {
    out.persons.push_back(track_to_message(kv.second, image_size, intrinsics, monocular, &result.stats));
  }
  const auto message_fill_end = SteadyClock::now();
  result.stats.message_fill_ms = elapsed_ms(message_fill_begin, message_fill_end);
  return result;
}

}  // namespace smart_follower_perception
