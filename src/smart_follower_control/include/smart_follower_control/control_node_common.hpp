#pragma once

#include <functional>
#include <memory>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "smart_follower_control/lifecycle_utils.hpp"

namespace smart_follower_control
{

double clamp_rate_hz(double value);
double clamp_non_negative(double value);
double clamp_positive(double value, double minimum);
int clamp_int_min(int value, int minimum);
rcl_interfaces::msg::SetParametersResult make_ok_result();

template<typename PublisherT>
bool begin_recreate_lifecycle_publisher(
  const std::shared_ptr<PublisherT> & publisher,
  bool preserve_activation,
  const std::function<void()> & before_deactivate = {})
{
  const bool was_active = preserve_activation && publisher_is_activated(publisher);
  if (was_active) {
    if (before_deactivate) {
      before_deactivate();
    }
    deactivate_publisher(publisher);
  }
  return was_active;
}

template<typename PublisherT>
void restore_lifecycle_publisher(const std::shared_ptr<PublisherT> & publisher, bool was_active)
{
  if (was_active) {
    activate_publisher(publisher);
  }
}

}  // namespace smart_follower_control
