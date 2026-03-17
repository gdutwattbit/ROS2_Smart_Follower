#include <gtest/gtest.h>

#include <memory>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "smart_follower_perception/frame_sync.hpp"

using smart_follower_perception::FrameSynchronizer;

namespace
{

FrameSynchronizer::Image::SharedPtr make_image(double t_sec)
{
  auto msg = std::make_shared<FrameSynchronizer::Image>();
  msg->header.stamp.sec = static_cast<int32_t>(t_sec);
  msg->header.stamp.nanosec = static_cast<uint32_t>((t_sec - static_cast<double>(msg->header.stamp.sec)) * 1e9);
  return msg;
}

FrameSynchronizer::CameraInfo::SharedPtr make_info(double t_sec)
{
  auto msg = std::make_shared<FrameSynchronizer::CameraInfo>();
  msg->header.stamp.sec = static_cast<int32_t>(t_sec);
  msg->header.stamp.nanosec = static_cast<uint32_t>((t_sec - static_cast<double>(msg->header.stamp.sec)) * 1e9);
  return msg;
}

}  // namespace

TEST(FrameSync, PopTripletWithinSlop)
{
  FrameSynchronizer sync;
  sync.configure(0.04, 6);

  auto color = make_image(1.000);
  auto depth = make_image(1.010);
  auto info = make_info(1.020);

  sync.push_color(color);
  sync.push_depth(depth);
  sync.push_info(info);

  FrameSynchronizer::Triplet triplet;
  ASSERT_TRUE(sync.pop_next(triplet));
  EXPECT_EQ(triplet.color.get(), color.get());
  EXPECT_EQ(triplet.depth.get(), depth.get());
  EXPECT_EQ(triplet.info.get(), info.get());
}

TEST(FrameSync, CacheTrimIncrementsDropped)
{
  FrameSynchronizer sync;
  sync.configure(0.04, 3);

  sync.push_color(make_image(1.0));
  sync.push_color(make_image(1.1));
  sync.push_color(make_image(1.2));
  sync.push_color(make_image(1.3));
  sync.push_color(make_image(1.4));

  EXPECT_EQ(sync.color_size(), 3u);
  EXPECT_EQ(sync.dropped_frames(), 2u);
}
