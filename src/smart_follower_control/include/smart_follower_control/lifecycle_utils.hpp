#pragma once

#include <algorithm>
#include <chrono>
#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace smart_follower_control
{

inline bool is_primary_active(rclcpp_lifecycle::LifecycleNode & node)
{
  return node.get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

inline std::chrono::milliseconds hz_to_period(double rate_hz)
{
  const double clamped_rate = std::max(1.0, rate_hz);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / clamped_rate));
}

template<typename PublisherT>
inline bool publisher_is_activated(const std::shared_ptr<PublisherT> & publisher)
{
  return publisher && publisher->is_activated();
}

template<typename PublisherT>
inline void activate_publisher(const std::shared_ptr<PublisherT> & publisher)
{
  if (publisher && !publisher->is_activated()) {
    publisher->on_activate();
  }
}

template<typename PublisherT>
inline void deactivate_publisher(const std::shared_ptr<PublisherT> & publisher)
{
  if (publisher && publisher->is_activated()) {
    publisher->on_deactivate();
  }
}

template<typename PublisherT, typename MessageT>
inline void publish_if_activated(const std::shared_ptr<PublisherT> & publisher, const MessageT & message)
{
  if (publisher_is_activated(publisher)) {
    publisher->publish(message);
  }
}

template<typename NodeT>
int run_lifecycle_node(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeT>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

}  // namespace smart_follower_control
