#include "smart_follower_perception/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "smart_follower_perception/assignment.hpp"

namespace smart_follower_perception
{

Track::Track()
: kf(kStateDim, kMeasureDim, 0, CV_32F)
{
}

void Tracker::configure(const TrackerConfig & config)
{
  config_ = config;
}

void Tracker::reset()
{
  tracks_.clear();
  memory_bank_.clear();
  next_track_id_ = 1;
  last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

std::optional<int> Tracker::try_recover_lock_from_memory(
  const Detection & det,
  const rclcpp::Time & now,
  int preferred_track_id) const
{
  if (!det.feature_valid) {
    return std::nullopt;
  }

  int best_id = -1;
  double best_sim = -1.0;
  for (const auto & entry : memory_bank_) {
    if (entry.expiry < now) {
      continue;
    }
    if (preferred_track_id >= 0 && entry.track_id != preferred_track_id) {
      continue;
    }
    const double sim = 1.0 - cosine_distance(entry.feature, det.feature);
    if (sim > best_sim) {
      best_sim = sim;
      best_id = entry.track_id;
    }
  }

  if (best_id < 0 && preferred_track_id < 0) {
    for (const auto & entry : memory_bank_) {
      if (entry.expiry < now) {
        continue;
      }
      const double sim = 1.0 - cosine_distance(entry.feature, det.feature);
      if (sim > best_sim) {
        best_sim = sim;
        best_id = entry.track_id;
      }
    }
  }

  if (best_id >= 0 && best_sim >= config_.reid_recover_threshold) {
    return best_id;
  }
  return std::nullopt;
}

Tracker::RunResult Tracker::run_tracking(
  std::vector<Detection> detections,
  const cv::Size & image_size,
  const rclcpp::Time & stamp,
  bool detections_available)
{
  RunResult result;

  float dt = 0.1F;
  if (last_stamp_.nanoseconds() > 0) {
    dt = std::max(1e-3F, static_cast<float>((stamp - last_stamp_).seconds()));
  }
  last_stamp_ = stamp;

  for (auto & kv : tracks_) {
    predict_track(kv.second, dt);
  }

  if (!detections_available) {
    update_memory_bank(stamp);
    return result;
  }

  std::vector<Detection> high_det;
  std::vector<Detection> low_det;
  for (const auto & det : detections) {
    if (det.confidence >= config_.high_score_threshold) {
      high_det.push_back(det);
    } else if (det.confidence >= config_.low_score_threshold) {
      low_det.push_back(det);
    }
  }


  if (tracks_.empty()) {
    for (const auto & det : high_det) {
      auto track = create_track(det, stamp);
      const int inserted_id = track.id;
      tracks_.insert({inserted_id, std::move(track)});
      if (det.recovered_track_id >= 0) {
        remove_memory_entry(det.recovered_track_id);
        result.recovered_track_id = inserted_id;
      }
    }
    update_memory_bank(stamp);
    return result;
  }

  std::vector<int> track_ids;
  track_ids.reserve(tracks_.size());
  for (const auto & kv : tracks_) {
    track_ids.push_back(kv.first);
  }

  std::vector<std::vector<double>> cost1(track_ids.size(), std::vector<double>(high_det.size(), 2.0));
  for (std::size_t i = 0; i < track_ids.size(); ++i) {
    const auto & track = tracks_.at(track_ids[i]);
    for (std::size_t j = 0; j < high_det.size(); ++j) {
      cost1[i][j] = full_cost(track, high_det[j], image_size.width, image_size.height);
    }
  }

  auto assign1 = solve_assignment(cost1, config_.assignment_threshold);
  for (const auto & match : assign1.matches) {
    update_track(tracks_.at(track_ids[match.first]), high_det[match.second], stamp);
  }

  std::vector<int> unmatched_track_ids;
  unmatched_track_ids.reserve(assign1.unmatched_rows.size());
  for (int idx : assign1.unmatched_rows) {
    unmatched_track_ids.push_back(track_ids[idx]);
  }

  std::vector<std::vector<double>> cost2(unmatched_track_ids.size(), std::vector<double>(low_det.size(), 2.0));
  for (std::size_t i = 0; i < unmatched_track_ids.size(); ++i) {
    const auto & track = tracks_.at(unmatched_track_ids[i]);
    for (std::size_t j = 0; j < low_det.size(); ++j) {
      cost2[i][j] = second_stage_cost(track, low_det[j]);
    }
  }

  auto assign2 = solve_assignment(cost2, config_.second_stage_threshold);
  for (const auto & match : assign2.matches) {
    update_track(tracks_.at(unmatched_track_ids[match.first]), low_det[match.second], stamp);
  }

  for (int det_idx : assign1.unmatched_cols) {
    auto track = create_track(high_det[det_idx], stamp);
    const int inserted_id = track.id;
    tracks_.insert({inserted_id, std::move(track)});
    if (high_det[det_idx].recovered_track_id >= 0) {
      remove_memory_entry(high_det[det_idx].recovered_track_id);
      result.recovered_track_id = inserted_id;
    }
  }

  std::unordered_map<int, bool> updated;
  for (const auto & match : assign1.matches) {
    updated[track_ids[match.first]] = true;
  }
  for (const auto & match : assign2.matches) {
    updated[unmatched_track_ids[match.first]] = true;
  }

  const rclcpp::Duration memory_ttl = rclcpp::Duration::from_seconds(config_.memory_sec);
  std::vector<int> to_remove;
  for (auto & kv : tracks_) {
    auto & track = kv.second;
    if (updated.find(kv.first) == updated.end()) {
      track.miss_count += 1;
      if (track.miss_count > config_.max_miss_frames) {
        if (track.feature_valid) {
          memory_bank_.push_back(MemoryEntry{kv.first, track.ema_feature, stamp + memory_ttl});
        }
        to_remove.push_back(kv.first);
      } else {
        track.state = smart_follower_msgs::msg::TrackedPerson::LOST;
      }
    }
  }

  for (int id : to_remove) {
    tracks_.erase(id);
  }

  update_memory_bank(stamp);
  return result;
}

const std::unordered_map<int, Track> & Tracker::tracks() const
{
  return tracks_;
}

const rclcpp::Time & Tracker::last_stamp() const
{
  return last_stamp_;
}

cv::Matx<float, kStateDim, kStateDim> Tracker::build_transition(float dt)
{
  cv::Matx<float, kStateDim, kStateDim> transition = cv::Matx<float, kStateDim, kStateDim>::eye();
  transition(0, 5) = dt;
  transition(1, 6) = dt;
  transition(2, 7) = dt;
  transition(3, 8) = dt;
  transition(4, 9) = dt;
  return transition;
}

Track Tracker::create_track(const Detection & det, const rclcpp::Time & stamp)
{
  Track track;
  if (det.recovered_track_id >= 0 && tracks_.find(det.recovered_track_id) == tracks_.end()) {
    track.id = det.recovered_track_id;
    next_track_id_ = std::max(next_track_id_, det.recovered_track_id + 1);
  } else {
    track.id = next_track_id_++;
  }
  track.state = smart_follower_msgs::msg::TrackedPerson::TENTATIVE;
  track.confidence = det.confidence;
  track.bbox = det.bbox;
  track.depth_m = det.depth_m;
  track.first_seen = stamp;
  track.last_seen = stamp;
  track.hit_count = 1;
  track.kf.transitionMatrix = cv::Mat(build_transition(1.0F));
  track.kf.measurementMatrix = cv::Mat::zeros(kMeasureDim, kStateDim, CV_32F);
  track.kf.measurementMatrix.at<float>(0, 0) = 1.0F;
  track.kf.measurementMatrix.at<float>(1, 1) = 1.0F;
  track.kf.measurementMatrix.at<float>(2, 3) = 1.0F;
  track.kf.measurementMatrix.at<float>(3, 4) = 1.0F;
  cv::setIdentity(track.kf.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(track.kf.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(track.kf.errorCovPost, cv::Scalar::all(1.0));

  track.kf.statePost.at<float>(0, 0) = det.bbox.x + det.bbox.width * 0.5F;
  track.kf.statePost.at<float>(1, 0) = det.bbox.y + det.bbox.height * 0.5F;
  track.kf.statePost.at<float>(2, 0) = det.depth_m;
  track.kf.statePost.at<float>(3, 0) = det.bbox.width;
  track.kf.statePost.at<float>(4, 0) = det.bbox.height;

  if (det.feature_valid) {
    track.feature_valid = true;
    track.ema_feature = det.feature;
    track.feature_buffer.push_back(det.feature);
  }
  return track;
}

void Tracker::predict_track(Track & track, float dt)
{
  track.kf.transitionMatrix = cv::Mat(build_transition(dt));
  cv::Mat prediction = track.kf.predict();
  const float cx = prediction.at<float>(0, 0);
  const float cy = prediction.at<float>(1, 0);
  const float width = std::max(2.0F, prediction.at<float>(3, 0));
  const float height = std::max(2.0F, prediction.at<float>(4, 0));
  track.depth_m = prediction.at<float>(2, 0);
  track.bbox = cv::Rect2f(cx - width * 0.5F, cy - height * 0.5F, width, height);
}

void Tracker::update_track(Track & track, const Detection & det, const rclcpp::Time & stamp)
{
  cv::Mat measurement(kMeasureDim, 1, CV_32F);
  measurement.at<float>(0, 0) = det.bbox.x + det.bbox.width * 0.5F;
  measurement.at<float>(1, 0) = det.bbox.y + det.bbox.height * 0.5F;
  measurement.at<float>(2, 0) = det.bbox.width;
  measurement.at<float>(3, 0) = det.bbox.height;
  track.kf.correct(measurement);

  if (det.depth_m >= config_.depth_min_m && det.depth_m <= config_.depth_max_m) {
    track.depth_m = det.depth_m;
    track.kf.statePost.at<float>(2, 0) = det.depth_m;
    track.invalid_depth_frames = 0;
  } else {
    track.invalid_depth_frames += 1;
  }

  track.bbox = det.bbox;
  track.confidence = det.confidence;
  track.hit_count += 1;
  track.miss_count = 0;
  track.last_seen = stamp;
  if (track.hit_count >= config_.min_confirm_hits) {
    track.state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
  }

  if (det.feature_valid) {
    if (!track.feature_valid) {
      track.ema_feature = det.feature;
      track.feature_valid = true;
    } else {
      for (int i = 0; i < kFeatureDim; ++i) {
        track.ema_feature[i] = static_cast<float>((1.0 - config_.ema_alpha) * track.ema_feature[i] + config_.ema_alpha * det.feature[i]);
      }
    }
    track.feature_buffer.push_back(det.feature);
    while (static_cast<int>(track.feature_buffer.size()) > config_.feature_buffer_size) {
      track.feature_buffer.pop_front();
    }
  }
}

double Tracker::appearance_cost(const Track & track, const Detection & det) const
{
  if (!track.feature_valid || !det.feature_valid) {
    return 1.0;
  }
  return std::min(1.0, cosine_distance(track.ema_feature, det.feature));
}

double Tracker::full_cost(const Track & track, const Detection & det, float img_w, float img_h) const
{
  const double iou_cost = 1.0 - bbox_iou(track.bbox, det.bbox);
  const double center_cost = normalized_center_distance(track.bbox, det.bbox, img_w, img_h);
  double depth_cost = 1.0;
  if (det.depth_m > 0.0F && track.depth_m > 0.0F) {
    const double depth_delta = std::abs(static_cast<double>(det.depth_m) - static_cast<double>(track.depth_m));
    depth_cost = clamp01(depth_delta / std::max(1e-3, static_cast<double>(config_.depth_norm_m)));
    if (depth_delta > config_.depth_gate_m) {
      return 2.0;
    }
  }
  return config_.weights.w_iou * iou_cost +
         config_.weights.w_center * center_cost +
         config_.weights.w_depth * depth_cost +
         config_.weights.w_appearance * appearance_cost(track, det);
}

double Tracker::second_stage_cost(const Track & track, const Detection & det) const
{
  const double iou_cost = 1.0 - bbox_iou(track.bbox, det.bbox);
  return 0.7 * iou_cost + 0.3 * appearance_cost(track, det);
}

void Tracker::update_memory_bank(const rclcpp::Time & now)
{
  while (!memory_bank_.empty() && memory_bank_.front().expiry < now) {
    memory_bank_.pop_front();
  }
}

void Tracker::remove_memory_entry(int track_id)
{
  memory_bank_.erase(
    std::remove_if(
      memory_bank_.begin(), memory_bank_.end(),
      [track_id](const MemoryEntry & entry) { return entry.track_id == track_id; }),
    memory_bank_.end());
}

}  // namespace smart_follower_perception
