#include <gtest/gtest.h>

#include "smart_follower_control/arbiter_state_machine.hpp"

using smart_follower_control::ArbiterMode;
using smart_follower_control::ArbiterThresholds;
using smart_follower_control::select_follow_mode;

TEST(ArbiterStateMachine, FollowsThresholdBands)
{
  ArbiterThresholds t;
  t.lost_time_normal_max = 0.2;
  t.lost_time_degraded_max = 0.6;
  t.lost_time_search_max = 2.0;

  EXPECT_EQ(select_follow_mode(0.1, t), ArbiterMode::FOLLOW_NORMAL);
  EXPECT_EQ(select_follow_mode(0.5, t), ArbiterMode::FOLLOW_DEGRADED);
  EXPECT_EQ(select_follow_mode(1.0, t), ArbiterMode::SEARCH);
  EXPECT_EQ(select_follow_mode(3.0, t), ArbiterMode::STOP);
}
