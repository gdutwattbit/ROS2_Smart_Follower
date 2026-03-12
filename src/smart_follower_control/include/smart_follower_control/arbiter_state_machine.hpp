#pragma once

#include <cstdint>

namespace smart_follower_control
{

enum class ArbiterMode : uint8_t
{
  FOLLOW_NORMAL = 0,
  FOLLOW_DEGRADED = 1,
  SEARCH = 2,
  AVOID = 3,
  STOP = 4
};

struct ArbiterThresholds
{
  double lost_time_normal_max{0.2};
  double lost_time_degraded_max{0.6};
  double lost_time_search_max{2.0};
};

inline ArbiterMode select_follow_mode(double age, const ArbiterThresholds & t)
{
  if (age <= t.lost_time_normal_max) {
    return ArbiterMode::FOLLOW_NORMAL;
  }
  if (age <= t.lost_time_degraded_max) {
    return ArbiterMode::FOLLOW_DEGRADED;
  }
  if (age <= t.lost_time_search_max) {
    return ArbiterMode::SEARCH;
  }
  return ArbiterMode::STOP;
}

}  // namespace smart_follower_control
