#pragma once

#include <deque>
#include <mutex>

#include <sensor_msgs/msg/image.hpp>

namespace smart_follower_perception
{

class FrameSynchronizer
{
public:
  using Image = sensor_msgs::msg::Image;

  struct Frame
  {
    Image::SharedPtr color;
  };

  void configure(int cache_size);
  void clear();
  void reset();

  void push_color(const Image::SharedPtr & msg);
  bool pop_next(Frame & frame);

  std::size_t color_size() const;
  std::size_t dropped_frames() const;

private:
  template<typename MsgT>
  struct CachedMessage
  {
    typename MsgT::SharedPtr msg;
  };

  template<typename MsgT>
  void trim_cache_to_limit(std::deque<CachedMessage<MsgT>> & cache);

  mutable std::mutex mutex_;
  int cache_size_{6};
  std::size_t dropped_frames_{0};
  std::deque<CachedMessage<Image>> color_cache_;
};

}  // namespace smart_follower_perception
