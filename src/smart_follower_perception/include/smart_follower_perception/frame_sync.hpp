#pragma once

#include <deque>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace smart_follower_perception
{

class FrameSynchronizer
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  struct Triplet
  {
    Image::SharedPtr color;
    Image::SharedPtr depth;
    CameraInfo::SharedPtr info;
  };

  void configure(double sync_slop, int cache_size);
  void clear();
  void reset();

  void push_color(const Image::SharedPtr & msg);
  void push_depth(const Image::SharedPtr & msg);
  void push_info(const CameraInfo::SharedPtr & msg);

  bool pop_next(Triplet & triplet);

  std::size_t color_size() const;
  std::size_t depth_size() const;
  std::size_t info_size() const;
  std::size_t dropped_frames() const;

private:
  template<typename MsgT>
  struct CachedMessage
  {
    typename MsgT::SharedPtr msg;
    rclcpp::Time stamp;
  };

  template<typename MsgT>
  void trim_cache_to_limit(std::deque<CachedMessage<MsgT>> & cache);

  template<typename MsgT>
  std::optional<std::size_t> find_best_match_index(
    const std::deque<CachedMessage<MsgT>> & cache,
    const rclcpp::Time & target_stamp) const;

  template<typename MsgT>
  void discard_stale_front_entries(
    std::deque<CachedMessage<MsgT>> & cache,
    const rclcpp::Time & reference_stamp);

  mutable std::mutex mutex_;
  double sync_slop_{0.04};
  int cache_size_{6};
  std::size_t dropped_frames_{0};
  std::deque<CachedMessage<Image>> color_cache_;
  std::deque<CachedMessage<Image>> depth_cache_;
  std::deque<CachedMessage<CameraInfo>> info_cache_;
};

}  // namespace smart_follower_perception
