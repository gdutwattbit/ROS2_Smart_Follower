#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "smart_follower_perception/constants.hpp"
#include "smart_follower_perception/frame_sync.hpp"
#include "smart_follower_perception/lock_manager.hpp"
#include "smart_follower_perception/perception_diagnostics.hpp"
#include "smart_follower_perception/perception_params.hpp"
#include "smart_follower_perception/perception_pipeline.hpp"
#include "smart_follower_perception/runtime.hpp"
#include "smart_follower_perception/tracker.hpp"

namespace smart_follower_perception
{

class PerceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  PerceptionNode()
  : rclcpp_lifecycle::LifecycleNode("perception_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    diagnostics_(this),
    pipeline_(
      get_logger(),
      *get_clock(),
      params_,
      stats_,
      tracker_,
      lock_manager_,
      yolo_,
      reid_,
      camera_model_,
      tf_buffer_)
  {
    ::smart_follower_perception::declare_parameters(*this, params_);
  }

private:
  void configure_modules_from_params()
  {
    TrackerConfig tracker_config;
    tracker_config.min_confirm_hits = params_.min_confirm_hits;
    tracker_config.max_miss_frames = params_.max_miss_frames;
    tracker_config.feature_buffer_size = params_.feature_buffer_size;
    tracker_config.memory_sec = params_.memory_sec;
    tracker_config.low_score_threshold = params_.low_score_threshold;
    tracker_config.high_score_threshold = params_.high_score_threshold;
    tracker_config.assignment_threshold = params_.assignment_threshold;
    tracker_config.second_stage_threshold = params_.second_stage_threshold;
    tracker_config.depth_gate_m = params_.depth_gate_m;
    tracker_config.depth_norm_m = params_.depth_norm_m;
    tracker_config.depth_min_m = params_.depth_min_m;
    tracker_config.depth_max_m = params_.depth_max_m;
    tracker_config.ema_alpha = params_.ema_alpha;
    tracker_config.reid_recover_threshold = params_.reid_recover_threshold;
    tracker_config.weights = params_.weights;
    tracker_.configure(tracker_config);

    LockConfig lock_config;
    lock_config.stable_frames = params_.lock_stable_frames;
    lock_config.hold_sec = params_.lock_hold_sec;
    lock_config.switch_sec = params_.lock_switch_sec;
    lock_config.center_roi_ratio = params_.lock_center_roi_ratio;
    lock_config.target_area_ratio = params_.lock_target_area_ratio;
    lock_manager_.configure(lock_config);

    frame_sync_.configure(params_.sync_slop, params_.sync_cache_size);
  }

  void configure_models()
  {
    params_.yolo_model_path = resolve_model_path(params_.yolo_model_path);
    params_.reid_model_path = resolve_model_path(params_.reid_model_path);
    yolo_.configure(
      params_.yolo_model_path,
      params_.yolo_input_w,
      params_.yolo_input_h,
      params_.person_class_id,
      params_.yolo_conf_threshold,
      params_.yolo_ort);
    reid_.configure(
      params_.reid_model_path,
      params_.reid_input_w,
      params_.reid_input_h,
      params_.reid_ort);
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation && person_pub_ && person_pub_->is_activated();
    if (was_active) {
      person_pub_->on_deactivate();
    }

    color_sub_.reset();
    depth_sub_.reset();
    info_sub_.reset();
    command_sub_.reset();
    person_pub_.reset();
    frame_sync_.clear();

    person_pub_ = this->create_publisher<smart_follower_msgs::msg::PersonPoseArray>(
      params_.person_pose_topic,
      rclcpp::SystemDefaultsQoS());

    command_sub_ = this->create_subscription<smart_follower_msgs::msg::FollowCommand>(
      params_.follow_command_topic,
      10,
      std::bind(&PerceptionNode::on_follow_command, this, std::placeholders::_1));

    color_sub_ = this->create_subscription<Image>(
      params_.color_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PerceptionNode::on_color_message, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<Image>(
      params_.depth_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PerceptionNode::on_depth_message, this, std::placeholders::_1));
    info_sub_ = this->create_subscription<CameraInfo>(
      params_.camera_info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PerceptionNode::on_info_message, this, std::placeholders::_1));

    if (was_active) {
      person_pub_->on_activate();
    }
  }

  void log_raw_input_status()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "[%s] raw_input color=%zu depth=%zu info=%zu cache=(%zu,%zu,%zu) last_stamp=(%.3f, %.3f, %.3f)",
      kRuntimeVersion,
      stats_.raw_color_count,
      stats_.raw_depth_count,
      stats_.raw_info_count,
      frame_sync_.color_size(),
      frame_sync_.depth_size(),
      frame_sync_.info_size(),
      PerceptionDiagnostics::stamp_seconds_or_negative(stats_.last_color_msg_stamp),
      PerceptionDiagnostics::stamp_seconds_or_negative(stats_.last_depth_msg_stamp),
      PerceptionDiagnostics::stamp_seconds_or_negative(stats_.last_info_msg_stamp));
  }

  void try_process_cached_frames()
  {
    std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_, std::try_to_lock);
    if (!dispatch_lock.owns_lock()) {
      return;
    }

    FrameSynchronizer::Triplet triplet;
    while (frame_sync_.pop_next(triplet)) {
      pipeline_.enqueue_synchronized_frame(
        SynchronizedFrame{triplet.color, triplet.depth, triplet.info},
        this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    }
  }

  void on_color_message(const Image::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    stats_.raw_color_count += 1;
    stats_.last_color_msg_stamp = rclcpp::Time(msg->header.stamp);
    frame_sync_.push_color(msg);
    log_raw_input_status();
    try_process_cached_frames();
  }

  void on_depth_message(const Image::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    stats_.raw_depth_count += 1;
    stats_.last_depth_msg_stamp = rclcpp::Time(msg->header.stamp);
    frame_sync_.push_depth(msg);
    log_raw_input_status();
    try_process_cached_frames();
  }

  void on_info_message(const CameraInfo::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    stats_.raw_info_count += 1;
    stats_.last_info_msg_stamp = rclcpp::Time(msg->header.stamp);
    frame_sync_.push_info(msg);
    log_raw_input_status();
    try_process_cached_frames();
  }

  void on_follow_command(const smart_follower_msgs::msg::FollowCommand::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (lock_manager_.handle_command(*msg, tracker_.last_stamp())) {
      tracker_.reset();
    }
  }

  void on_worker_result_timer()
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    pipeline_.consume_ready_result(
      person_pub_,
      [this]() { return this->now(); },
      [this]() { diagnostics_.force_update(); });
  }

  void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stats_.fill_status(
      stat,
      tracker_.tracks().size(),
      frame_sync_.dropped_frames(),
      lock_manager_.lock_id(),
      lock_manager_.lock_state(),
      lock_manager_.last_lock_confirmed_time(),
      yolo_.ready(),
      reid_.ready(),
      this->now());
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> & parameters)
  {
    PerceptionParams candidate = params_;
    for (const auto & parameter : parameters) {
      ::smart_follower_perception::apply_parameter_override(candidate, parameter);
    }
    const bool was_active = this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    if (was_active && result_timer_) {
      result_timer_->cancel();
    }
    pipeline_.stop_detection_worker();
    pipeline_.clear_async_state();

    params_ = candidate;
    configure_modules_from_params();
    configure_models();
    recreate_interfaces(was_active);

    if (was_active) {
      pipeline_.start_detection_worker();
      if (result_timer_) {
        result_timer_->reset();
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] parameters hot-reloaded: color=%s depth=%s info=%s person_pose=%s yolo=%s reid=%s sync_slop=%.3f yolo_ort=(intra=%d inter=%d mode=%s) reid_ort=(intra=%d inter=%d mode=%s)",
      kRuntimeVersion,
      params_.color_topic.c_str(),
      params_.depth_topic.c_str(),
      params_.camera_info_topic.c_str(),
      params_.person_pose_topic.c_str(),
      params_.yolo_model_path.c_str(),
      params_.reid_model_path.c_str(),
      params_.sync_slop,
      params_.yolo_ort.intra_op_num_threads,
      params_.yolo_ort.inter_op_num_threads,
      params_.yolo_ort.execution_mode_parallel ? "parallel" : "sequential",
      params_.reid_ort.intra_op_num_threads,
      params_.reid_ort.inter_op_num_threads,
      params_.reid_ort.execution_mode_parallel ? "parallel" : "sequential");

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    ::smart_follower_perception::load_parameters(*this, params_);
    configure_modules_from_params();
    configure_models();
    recreate_interfaces(false);
    pipeline_.clear_async_state();

    if (!result_timer_) {
      result_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PerceptionNode::on_worker_result_timer, this));
      result_timer_->cancel();
    }

    diagnostics_.setHardwareID("smart_follower_perception");
    if (!diagnostics_registered_) {
      diagnostics_.add("perception_status", this, &PerceptionNode::diagnostics_callback);
      diagnostics_registered_ = true;
    }

    if (!param_callback_handle_) {
      param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PerceptionNode::on_parameters_set, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] Configured perception node. YOLO ready=%d ReID ready=%d yolo_input=%dx%d reid_input=%dx%d process_every_n_frames=%d detect_every_n_frames=%d yolo_ort=(intra=%d inter=%d mode=%s) reid_ort=(intra=%d inter=%d mode=%s)",
      kRuntimeVersion,
      yolo_.ready(),
      reid_.ready(),
      params_.yolo_input_w,
      params_.yolo_input_h,
      params_.reid_input_w,
      params_.reid_input_h,
      params_.process_every_n_frames,
      params_.detect_every_n_frames,
      params_.yolo_ort.intra_op_num_threads,
      params_.yolo_ort.inter_op_num_threads,
      params_.yolo_ort.execution_mode_parallel ? "parallel" : "sequential",
      params_.reid_ort.intra_op_num_threads,
      params_.reid_ort.inter_op_num_threads,
      params_.reid_ort.execution_mode_parallel ? "parallel" : "sequential");
    RCLCPP_INFO(
      get_logger(),
      "[%s] input topics color=%s depth=%s info=%s person_pose=%s sync_slop=%.3f cache_size=%d",
      kRuntimeVersion,
      params_.color_topic.c_str(),
      params_.depth_topic.c_str(),
      params_.camera_info_topic.c_str(),
      params_.person_pose_topic.c_str(),
      params_.sync_slop,
      params_.sync_cache_size);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (person_pub_) {
      person_pub_->on_activate();
    }
    pipeline_.clear_async_state();
    pipeline_.start_detection_worker();
    if (result_timer_) {
      result_timer_->reset();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (result_timer_) {
      result_timer_->cancel();
    }
    pipeline_.stop_detection_worker();
    pipeline_.clear_async_state();
    if (person_pub_) {
      person_pub_->on_deactivate();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    if (result_timer_) {
      result_timer_->cancel();
      result_timer_.reset();
    }
    pipeline_.stop_detection_worker();
    pipeline_.clear_async_state();
    color_sub_.reset();
    depth_sub_.reset();
    info_sub_.reset();
    command_sub_.reset();
    person_pub_.reset();
    frame_sync_.reset();
    tracker_.reset();
    lock_manager_.reset();
    stats_.reset();
    return CallbackReturn::SUCCESS;
  }

  PerceptionParams params_;
  PerceptionDiagnostics stats_;
  FrameSynchronizer frame_sync_;
  Tracker tracker_;
  LockManager lock_manager_;
  YoloDetector yolo_;
  ReidExtractor reid_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  diagnostic_updater::Updater diagnostics_;
  image_geometry::PinholeCameraModel camera_model_;
  PerceptionPipeline pipeline_;

  rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr person_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  bool diagnostics_registered_{false};
  std::mutex dispatch_mutex_;
  rclcpp::TimerBase::SharedPtr result_timer_;
};

}  // namespace smart_follower_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_perception::PerceptionNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
