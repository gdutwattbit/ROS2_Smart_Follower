#include <gtest/gtest.h>

#include <memory>

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

}  // namespace

TEST(FrameSync, PopsQueuedColorFramesInOrder)
{
  FrameSynchronizer sync;
  sync.configure(6);

  auto color1 = make_image(1.000);
  auto color2 = make_image(1.010);
  sync.push_color(color1);
  sync.push_color(color2);

  FrameSynchronizer::Frame frame;
  ASSERT_TRUE(sync.pop_next(frame));
  EXPECT_EQ(frame.color.get(), color1.get());
  ASSERT_TRUE(sync.pop_next(frame));
  EXPECT_EQ(frame.color.get(), color2.get());
  EXPECT_FALSE(sync.pop_next(frame));
}

TEST(FrameSync, CacheTrimIncrementsDropped)
{
  FrameSynchronizer sync;
  sync.configure(3);

  sync.push_color(make_image(1.0));
  sync.push_color(make_image(1.1));
  sync.push_color(make_image(1.2));
  sync.push_color(make_image(1.3));
  sync.push_color(make_image(1.4));

  EXPECT_EQ(sync.color_size(), 3u);
  EXPECT_EQ(sync.dropped_frames(), 2u);
}
