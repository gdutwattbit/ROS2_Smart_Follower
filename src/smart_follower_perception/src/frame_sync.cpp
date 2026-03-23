#include "smart_follower_perception/frame_sync.hpp"

#include <algorithm>

namespace smart_follower_perception
{

void FrameSynchronizer::configure(int cache_size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cache_size_ = std::max(1, cache_size);
}

void FrameSynchronizer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.clear();
}

void FrameSynchronizer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.clear();
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

void FrameSynchronizer::push_color(const Image::SharedPtr & msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  color_cache_.push_back(CachedMessage<Image>{msg});
  trim_cache_to_limit(color_cache_);
}

bool FrameSynchronizer::pop_next(Frame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (color_cache_.empty()) {
    return false;
  }

  frame.color = color_cache_.front().msg;
  color_cache_.pop_front();
  return true;
}

std::size_t FrameSynchronizer::color_size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return color_cache_.size();
}

std::size_t FrameSynchronizer::dropped_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_frames_;
}

template void FrameSynchronizer::trim_cache_to_limit(std::deque<CachedMessage<Image>> & cache);

}  // namespace smart_follower_perception
