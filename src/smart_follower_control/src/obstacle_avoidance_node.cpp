#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>



namespace smart_follower_control
{
namespace
{
float percentile(std::vector<float> data, float q)
{
  if (data.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  q = std::max(0.0F, std::min(1.0F, q));
  std::sort(data.begin(), data.end());
  float idxf = q * static_cast<float>(data.size() - 1);
  std::size_t lo = static_cast<std::size_t>(std::floor(idxf));
  std::size_t hi = static_cast<std::size_t>(std::ceil(idxf));
  if (lo == hi) {
    return data[lo];
  }
  float t = idxf - static_cast<float>(lo);
  return data[lo] * (1.0F - t) + data[hi] * t;
}
}  // namespace


class ObstacleAvoidanceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  ObstacleAvoidanceNode()
  : rclcpp_lifecycle::LifecycleNode("obstacle_avoidance_node")
  {
    declare_parameter("left_range_topic", std::string("left_ultrasonic/range"));
    declare_parameter("right_range_topic", std::string("right_ultrasonic/range"));
    declare_parameter("depth_topic", std::string("/camera/depth/image_raw"));
    declare_parameter("cmd_vel_input_topic", std::string("/cmd_vel"));
    declare_parameter("cmd_vel_avoid_topic", std::string("cmd_vel_avoid"));

    declare_parameter("rate", 20.0);
    declare_parameter("d_min", 0.12);
    declare_parameter("t_react", 0.20);
    declare_parameter("a_brake", 0.8);
    declare_parameter("margin", 0.08);
    declare_parameter("exit_margin", 0.08);
    declare_parameter("depth_roi_width_ratio", 0.4);
    declare_parameter("depth_roi_height_ratio", 0.35);
    declare_parameter("depth_percentile", 0.05);

    declare_parameter("turn_speed", 0.5);
    declare_parameter("slow_turn_speed", 0.25);
    declare_parameter("back_speed", -0.15);
  }

private:
  struct Params
  {
    std::string left_topic{"left_ultrasonic/range"};
    std::string right_topic{"right_ultrasonic/range"};
    std::string depth_topic{"/camera/depth/image_raw"};
    std::string cmd_vel_input_topic{"/cmd_vel"};
    std::string cmd_vel_avoid_topic{"cmd_vel_avoid"};

    double rate{20.0};
    double d_min{0.12};
    double t_react{0.20};
    double a_brake{0.8};
    double margin{0.08};
    double exit_margin{0.08};
    double depth_roi_width_ratio{0.4};
    double depth_roi_height_ratio{0.35};
    double depth_percentile{0.05};

    double turn_speed{0.5};
    double slow_turn_speed{0.25};
    double back_speed{-0.15};
  } p_;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr avoid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_{this};

  double left_dist_{std::numeric_limits<double>::infinity()};
  double right_dist_{std::numeric_limits<double>::infinity()};
  double depth_dist_{std::numeric_limits<double>::infinity()};
  double current_speed_{0.0};

  rclcpp::Time left_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time right_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time depth_stamp_{0, 0, RCL_ROS_TIME};

  void load_parameters()
  {
    p_.left_topic = get_parameter("left_range_topic").as_string();
    p_.right_topic = get_parameter("right_range_topic").as_string();
    p_.depth_topic = get_parameter("depth_topic").as_string();
    p_.cmd_vel_input_topic = get_parameter("cmd_vel_input_topic").as_string();
    p_.cmd_vel_avoid_topic = get_parameter("cmd_vel_avoid_topic").as_string();

    p_.rate = get_parameter("rate").as_double();
    p_.d_min = get_parameter("d_min").as_double();
    p_.t_react = get_parameter("t_react").as_double();
    p_.a_brake = get_parameter("a_brake").as_double();
    p_.margin = get_parameter("margin").as_double();
    p_.exit_margin = get_parameter("exit_margin").as_double();
    p_.depth_roi_width_ratio = get_parameter("depth_roi_width_ratio").as_double();
    p_.depth_roi_height_ratio = get_parameter("depth_roi_height_ratio").as_double();
    p_.depth_percentile = get_parameter("depth_percentile").as_double();
    p_.turn_speed = get_parameter("turn_speed").as_double();
    p_.slow_turn_speed = get_parameter("slow_turn_speed").as_double();
    p_.back_speed = get_parameter("back_speed").as_double();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();

    left_sub_ = create_subscription<sensor_msgs::msg::Range>(
      p_.left_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        if (!msg) return;
        left_dist_ = msg->range;
        left_stamp_ = rclcpp::Time(msg->header.stamp);
      });

    right_sub_ = create_subscription<sensor_msgs::msg::Range>(
      p_.right_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr msg) {
        if (!msg) return;
        right_dist_ = msg->range;
        right_stamp_ = rclcpp::Time(msg->header.stamp);
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      p_.depth_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidanceNode::on_depth, this, std::placeholders::_1));

    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_input_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          current_speed_ = std::abs(msg->linear.x);
        }
      });

    avoid_pub_ = create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_avoid_topic, 10);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, p_.rate))),
      std::bind(&ObstacleAvoidanceNode::on_timer, this));

    diagnostics_.setHardwareID("smart_follower_avoidance");
    diagnostics_.add("avoidance_status", this, &ObstacleAvoidanceNode::diag_callback);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    avoid_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (avoid_pub_) {
      avoid_pub_->on_deactivate();
    }
    publish_zero();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    left_sub_.reset();
    right_sub_.reset();
    depth_sub_.reset();
    vel_sub_.reset();
    avoid_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    cv::Mat depth;
    try {
      if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
      } else {
        depth = cv_bridge::toCvShare(msg)->image;
      }
    } catch (const cv_bridge::Exception &) {
      return;
    }

    const int w = depth.cols;
    const int h = depth.rows;
    if (w <= 0 || h <= 0) {
      return;
    }

    const int roi_w = static_cast<int>(w * p_.depth_roi_width_ratio);
    const int roi_h = static_cast<int>(h * p_.depth_roi_height_ratio);
    const int x0 = std::max(0, (w - roi_w) / 2);
    const int y0 = std::max(0, (h - roi_h) / 2);
    const int x1 = std::min(w - 1, x0 + roi_w);
    const int y1 = std::min(h - 1, y0 + roi_h);

    std::vector<float> valid;
    valid.reserve(static_cast<std::size_t>(roi_w * roi_h));
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        float m = 0.0F;
        if (depth.type() == CV_16UC1) {
          uint16_t mm = depth.at<uint16_t>(y, x);
          if (mm == 0) {
            continue;
          }
          m = static_cast<float>(mm) * 0.001F;
        } else if (depth.type() == CV_32FC1) {
          m = depth.at<float>(y, x);
        } else {
          continue;
        }

        if (std::isfinite(m) && m > 0.05F && m < 8.0F) {
          valid.push_back(m);
        }
      }
    }

    if (!valid.empty()) {
      depth_dist_ = percentile(valid, static_cast<float>(p_.depth_percentile));
      depth_stamp_ = rclcpp::Time(msg->header.stamp);
    }
  }

  void on_timer()
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    const auto t = now();
    const auto stale = [&](const rclcpp::Time & stamp) {
      return stamp.nanoseconds() == 0 || (t - stamp).seconds() > 0.5;
    };

    double left = stale(left_stamp_) ? std::numeric_limits<double>::infinity() : left_dist_;
    double right = stale(right_stamp_) ? std::numeric_limits<double>::infinity() : right_dist_;
    double depth = stale(depth_stamp_) ? std::numeric_limits<double>::infinity() : depth_dist_;

    const double d_safe = p_.d_min + current_speed_ * p_.t_react +
                          (current_speed_ * current_speed_) / (2.0 * std::max(1e-3, p_.a_brake)) +
                          p_.margin;
    const double d_enter = d_safe;
    const double d_exit = d_safe + p_.exit_margin;
    const double d_danger = std::max(0.18, 0.6 * d_safe);

    const double fused_min = std::min({left, right, depth});

    geometry_msgs::msg::Twist out;

    if (fused_min > d_exit) {
      out = geometry_msgs::msg::Twist();
    } else if (left < d_danger && right < d_danger) {
      out.linear.x = p_.back_speed;
      out.angular.z = 0.0;
    } else if (left < d_danger && right > d_enter) {
      out.linear.x = 0.0;
      out.angular.z = -std::abs(p_.turn_speed);
    } else if (right < d_danger && left > d_enter) {
      out.linear.x = 0.0;
      out.angular.z = std::abs(p_.turn_speed);
    } else if (fused_min < d_enter) {
      out.linear.x = 0.0;
      out.angular.z = (left < right) ? -std::abs(p_.slow_turn_speed) : std::abs(p_.slow_turn_speed);
    }

    if (avoid_pub_ && avoid_pub_->is_activated()) {
      avoid_pub_->publish(out);
    }

    diagnostics_.force_update();
  }

  void publish_zero()
  {
    if (avoid_pub_ && avoid_pub_->is_activated()) {
      avoid_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  void diag_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.add("left_dist", left_dist_);
    stat.add("right_dist", right_dist_);
    stat.add("depth_dist", depth_dist_);
    stat.add("current_speed", current_speed_);
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Obstacle avoidance active");
  }
};

}  // namespace smart_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_control::ObstacleAvoidanceNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}






