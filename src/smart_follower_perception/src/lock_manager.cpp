#include "smart_follower_perception/lock_manager.hpp"

#include <cmath>
#include <limits>

namespace smart_follower_perception
{

void LockManager::configure(const LockConfig & config)
{
  config_ = config;
}

void LockManager::reset()
{
  lock_id_ = -1;
  lock_state_ = smart_follower_msgs::msg::PersonPoseArray::IDLE;
  last_lock_confirmed_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  pending_lock_request_ = false;
  pending_stable_track_ = -1;
  pending_stable_count_ = 0;
}

bool LockManager::handle_command(
  const smart_follower_msgs::msg::FollowCommand & command,
  const rclcpp::Time & current_stamp)
{
  switch (command.command) {
    case smart_follower_msgs::msg::FollowCommand::LOCK:
      if (command.target_id >= 0) {
        lock_id_ = command.target_id;
        lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
        last_lock_confirmed_time_ = current_stamp;
        pending_lock_request_ = false;
        pending_stable_track_ = -1;
        pending_stable_count_ = 0;
      } else {
        pending_lock_request_ = true;
        pending_stable_track_ = -1;
        pending_stable_count_ = 0;
      }
      return false;
    case smart_follower_msgs::msg::FollowCommand::UNLOCK:
      reset();
      return false;
    case smart_follower_msgs::msg::FollowCommand::RESET:
      reset();
      return true;
    case smart_follower_msgs::msg::FollowCommand::ESTOP:
      lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOST;
      return false;
    default:
      return false;
  }
}

void LockManager::update(
  const std::unordered_map<int, Track> & tracks,
  const cv::Size & image_size,
  const rclcpp::Time & stamp)
{
  if (pending_lock_request_) {
    const double lost_for = last_lock_confirmed_time_.nanoseconds() > 0 ?
      (stamp - last_lock_confirmed_time_).seconds() :
      std::numeric_limits<double>::infinity();
    const bool allow_switch = lock_id_ < 0 || lost_for >= config_.switch_sec;
    int best_track = -1;
    double best_score = -1.0;
    const float roi_w = static_cast<float>(image_size.width) * config_.center_roi_ratio;
    const float roi_h = static_cast<float>(image_size.height) * config_.center_roi_ratio;
    const cv::Rect2f roi(
      (image_size.width - roi_w) * 0.5F,
      (image_size.height - roi_h) * 0.5F,
      roi_w,
      roi_h);

    if (allow_switch) {
      for (const auto & [id, track] : tracks) {
        if (track.state != smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
          continue;
        }

        cv::Point2f center(track.bbox.x + track.bbox.width * 0.5F, track.bbox.y + track.bbox.height * 0.5F);
        if (!roi.contains(center)) {
          continue;
        }

        const float uc = static_cast<float>(image_size.width) * 0.5F;
        const float vc = static_cast<float>(image_size.height) * 0.5F;
        const float rmax = static_cast<float>(image_size.width) * 0.5F;
        const float dist = std::hypot(center.x - uc, center.y - vc);
        const double center_score = 1.0 - std::min(1.0, static_cast<double>(dist / std::max(1.0F, rmax)));

        const double area = static_cast<double>(track.bbox.area());
        const double target_area = static_cast<double>(image_size.width * image_size.height) * config_.target_area_ratio;
        const double area_score = std::min(area / target_area, target_area / std::max(1.0, area));
        const double score = 0.5 * center_score + 0.3 * area_score + 0.2 * track.confidence;
        if (score > best_score) {
          best_score = score;
          best_track = id;
        }
      }
    }

    if (best_track >= 0) {
      if (pending_stable_track_ == best_track) {
        pending_stable_count_++;
      } else {
        pending_stable_track_ = best_track;
        pending_stable_count_ = 1;
      }
    } else {
      pending_stable_track_ = -1;
      pending_stable_count_ = 0;
    }

    if (pending_stable_count_ >= config_.stable_frames && pending_stable_track_ >= 0) {
      lock_id_ = pending_stable_track_;
      lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
      pending_lock_request_ = false;
      pending_stable_track_ = -1;
      pending_stable_count_ = 0;
    }
  }

  if (lock_id_ >= 0) {
    auto it = tracks.find(lock_id_);
    if (it != tracks.end() && it->second.state == smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
      lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
      last_lock_confirmed_time_ = stamp;
    } else {
      const double lost_for = last_lock_confirmed_time_.nanoseconds() > 0 ?
        (stamp - last_lock_confirmed_time_).seconds() :
        std::numeric_limits<double>::infinity();
      lock_state_ = lost_for <= config_.hold_sec ?
        smart_follower_msgs::msg::PersonPoseArray::LOCKED :
        smart_follower_msgs::msg::PersonPoseArray::LOST;
    }
  } else {
    lock_state_ = smart_follower_msgs::msg::PersonPoseArray::IDLE;
  }
}

void LockManager::set_lock_id(int track_id)
{
  lock_id_ = track_id;
}

int LockManager::lock_id() const
{
  return lock_id_;
}

uint8_t LockManager::lock_state() const
{
  return lock_state_;
}

const rclcpp::Time & LockManager::last_lock_confirmed_time() const
{
  return last_lock_confirmed_time_;
}

}  // namespace smart_follower_perception
