#pragma once

#include <array>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smart_follower_msgs/msg/tracked_person.hpp>

#include "smart_follower_perception/constants.hpp"
#include "smart_follower_perception/tracking_utils.hpp"

namespace smart_follower_perception
{

struct MemoryEntry
{
  int track_id{-1};
  std::array<float, kFeatureDim> feature{};
  rclcpp::Time expiry{0, 0, RCL_ROS_TIME};
};

struct Track
{
  int id{-1};
  uint8_t state{smart_follower_msgs::msg::TrackedPerson::TENTATIVE};
  float confidence{0.0F};
  cv::KalmanFilter kf;
  cv::Rect2f bbox;
  float depth_m{0.0F};
  int invalid_depth_frames{0};
  int hit_count{0};
  int miss_count{0};
  rclcpp::Time first_seen{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  std::array<float, kFeatureDim> ema_feature{};
  bool feature_valid{false};
  std::deque<std::array<float, kFeatureDim>> feature_buffer;

  Track();
};

struct CostWeights
{
  double w_iou{0.3};
  double w_center{0.2};
  double w_depth{0.1};
  double w_appearance{0.4};
};

struct TrackerConfig
{
  int min_confirm_hits{3};
  int max_miss_frames{10};
  int feature_buffer_size{20};
  float memory_sec{30.0F};
  float low_score_threshold{0.1F};
  float high_score_threshold{0.5F};
  float assignment_threshold{0.7F};
  float second_stage_threshold{0.8F};
  float depth_gate_m{1.0F};
  float depth_norm_m{2.0F};
  float depth_min_m{0.2F};
  float depth_max_m{4.0F};
  float ema_alpha{0.2F};
  float reid_recover_threshold{0.70F};
  CostWeights weights;
};

class Tracker
{
public:
  struct RunResult
  {
    std::optional<int> recovered_track_id;
  };

  void configure(const TrackerConfig & config);
  void reset();

  std::optional<int> try_recover_lock_from_memory(
    const Detection & det,
    const rclcpp::Time & now,
    int preferred_track_id) const;

  RunResult run_tracking(
    std::vector<Detection> detections,
    const cv::Size & image_size,
    const rclcpp::Time & stamp,
    bool detections_available);

  const std::unordered_map<int, Track> & tracks() const;
  const rclcpp::Time & last_stamp() const;

private:
  static cv::Matx<float, kStateDim, kStateDim> build_transition(float dt);

  Track create_track(const Detection & det, const rclcpp::Time & stamp);
  void predict_track(Track & track, float dt);
  void update_track(Track & track, const Detection & det, const rclcpp::Time & stamp);
  double appearance_cost(const Track & track, const Detection & det) const;
  double full_cost(const Track & track, const Detection & det, float img_w, float img_h) const;
  double second_stage_cost(const Track & track, const Detection & det) const;
  void update_memory_bank(const rclcpp::Time & now);
  void remove_memory_entry(int track_id);

  TrackerConfig config_;
  std::unordered_map<int, Track> tracks_;
  std::deque<MemoryEntry> memory_bank_;
  int next_track_id_{1};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace smart_follower_perception
