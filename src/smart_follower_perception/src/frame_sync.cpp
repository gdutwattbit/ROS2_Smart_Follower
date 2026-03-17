#include "smart_follower_perception/frame_sync.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace smart_follower_perception
{

void FrameSynchronizer::configure(double sync_slop, int cache_size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sync_slop_ = sync_slop;
  cache_size_ = std::max(3, cache_size);
}

void FrameSynchronizer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.clear();
  depth_cache_.clear();
  info_cache_.clear();
}

void FrameSynchronizer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.clear();
  depth_cache_.clear();
  info_cache_.clear();
  dropped_frames_ = 0;
}

template<typename MsgT>
void FrameSynchronizer::trim_cache_to_limit(std::deque<CachedMessage<MsgT>> & cache)
{
  while (static_cast<int>(cache.size()) > cache_size_) {
    cache.pop_front();
    dropped_frames_ += 1;
  }
}

template<typename MsgT>
std::optional<std::size_t> FrameSynchronizer::find_best_match_index(
  const std::deque<CachedMessage<MsgT>> & cache,
  const rclcpp::Time & target_stamp) const
{
  std::optional<std::size_t> best_index;
  double best_diff = sync_slop_ + 1e-9;
  for (std::size_t i = 0; i < cache.size(); ++i) {
    const double diff = std::abs((cache[i].stamp - target_stamp).seconds());
    if (diff <= sync_slop_ && diff < best_diff) {
      best_diff = diff;
      best_index = i;
    }
  }
  return best_index;
}

template<typename MsgT>
void FrameSynchronizer::discard_stale_front_entries(
  std::deque<CachedMessage<MsgT>> & cache,
  const rclcpp::Time & reference_stamp)
{
  while (!cache.empty() && (reference_stamp - cache.front().stamp).seconds() > sync_slop_) {
    cache.pop_front();
    dropped_frames_ += 1;
  }
}

void FrameSynchronizer::push_color(const Image::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.push_back(CachedMessage<Image>{msg, rclcpp::Time(msg->header.stamp)});
  trim_cache_to_limit(color_cache_);
}

void FrameSynchronizer::push_depth(const Image::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  depth_cache_.push_back(CachedMessage<Image>{msg, rclcpp::Time(msg->header.stamp)});
  trim_cache_to_limit(depth_cache_);
}

void FrameSynchronizer::push_info(const CameraInfo::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  info_cache_.push_back(CachedMessage<CameraInfo>{msg, rclcpp::Time(msg->header.stamp)});
  trim_cache_to_limit(info_cache_);
}

bool FrameSynchronizer::pop_next(Triplet & triplet)
{
  std::lock_guard<std::mutex> lock(mutex_);

  while (true) {
    if (color_cache_.empty() || depth_cache_.empty() || info_cache_.empty()) {
      return false;
    }

    const rclcpp::Time color_stamp = color_cache_.front().stamp;
    discard_stale_front_entries(depth_cache_, color_stamp);
    discard_stale_front_entries(info_cache_, color_stamp);
    if (color_cache_.empty() || depth_cache_.empty() || info_cache_.empty()) {
      return false;
    }

    const auto depth_idx = find_best_match_index(depth_cache_, color_stamp);
    const auto info_idx = find_best_match_index(info_cache_, color_stamp);
    if (depth_idx.has_value() && info_idx.has_value()) {
      triplet.color = color_cache_.front().msg;
      triplet.depth = depth_cache_.at(*depth_idx).msg;
      triplet.info = info_cache_.at(*info_idx).msg;
      color_cache_.pop_front();
      depth_cache_.erase(depth_cache_.begin() + static_cast<std::ptrdiff_t>(*depth_idx));
      info_cache_.erase(info_cache_.begin() + static_cast<std::ptrdiff_t>(*info_idx));
      return true;
    }

    const bool depth_too_old = (depth_cache_.back().stamp - color_stamp).seconds() < -sync_slop_;
    const bool info_too_old = (info_cache_.back().stamp - color_stamp).seconds() < -sync_slop_;
    if (depth_too_old || info_too_old) {
      color_cache_.pop_front();
      dropped_frames_ += 1;
      continue;
    }
    return false;
  }
}

std::size_t FrameSynchronizer::color_size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return color_cache_.size();
}

std::size_t FrameSynchronizer::depth_size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return depth_cache_.size();
}

std::size_t FrameSynchronizer::info_size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return info_cache_.size();
}

std::size_t FrameSynchronizer::dropped_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_frames_;
}

template void FrameSynchronizer::trim_cache_to_limit(std::deque<CachedMessage<Image>> & cache);
template void FrameSynchronizer::trim_cache_to_limit(std::deque<CachedMessage<CameraInfo>> & cache);
template std::optional<std::size_t> FrameSynchronizer::find_best_match_index(
  const std::deque<CachedMessage<Image>> & cache,
  const rclcpp::Time & target_stamp) const;
template std::optional<std::size_t> FrameSynchronizer::find_best_match_index(
  const std::deque<CachedMessage<CameraInfo>> & cache,
  const rclcpp::Time & target_stamp) const;
template void FrameSynchronizer::discard_stale_front_entries(
  std::deque<CachedMessage<Image>> & cache,
  const rclcpp::Time & reference_stamp);
template void FrameSynchronizer::discard_stale_front_entries(
  std::deque<CachedMessage<CameraInfo>> & cache,
  const rclcpp::Time & reference_stamp);

}  // namespace smart_follower_perception
