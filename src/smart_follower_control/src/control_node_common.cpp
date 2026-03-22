#include "smart_follower_control/control_node_common.hpp"

#include <algorithm>

namespace smart_follower_control
{

double clamp_rate_hz(double value)
{
  return std::max(1.0, value);
}

double clamp_non_negative(double value)
{
  return std::max(0.0, value);
}

double clamp_positive(double value, double minimum)
{
  return std::max(minimum, value);
}

int clamp_int_min(int value, int minimum)
{
  return std::max(minimum, value);
}

rcl_interfaces::msg::SetParametersResult make_ok_result()
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";
  return result;
}

}  // namespace smart_follower_control
