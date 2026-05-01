#pragma once

#include <cstdint>

namespace smart_follower_control
{

enum class ArbiterMode : uint8_t
{
  FOLLOW = 0,
  AVOID = 1,
  STOP = 2
};

}  // namespace smart_follower_control
