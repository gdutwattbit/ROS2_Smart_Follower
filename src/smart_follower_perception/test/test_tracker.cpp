#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "smart_follower_perception/tracker.hpp"

using smart_follower_perception::Detection;
using smart_follower_perception::Tracker;
using smart_follower_perception::TrackerConfig;

namespace
{

Detection make_det(const cv::Rect2f & bbox, float conf)
{
  Detection d;
  d.bbox = bbox;
  d.confidence = conf;
  d.depth_m = 1.0F;
  d.feature_valid = false;
  return d;
}

}  // namespace

TEST(Tracker, InitializesTracksWhenEmpty)
{
  Tracker tracker;
  TrackerConfig cfg;
  cfg.min_confirm_hits = 2;
  cfg.high_score_threshold = 0.5F;
  cfg.low_score_threshold = 0.1F;
  tracker.configure(cfg);

  std::vector<Detection> dets;
  dets.push_back(make_det(cv::Rect2f(100, 100, 50, 120), 0.9F));

  tracker.run_tracking(dets, cv::Size(640, 480), rclcpp::Time(1, 0, RCL_ROS_TIME), true);
  ASSERT_EQ(tracker.tracks().size(), 1u);

  // second hit -> confirmed
  dets[0].bbox.x += 2.0F;
  tracker.run_tracking(dets, cv::Size(640, 480), rclcpp::Time(1, 10000000, RCL_ROS_TIME), true);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().begin()->second.state, smart_follower_msgs::msg::TrackedPerson::CONFIRMED);
}
