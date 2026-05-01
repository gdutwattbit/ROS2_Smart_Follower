#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>

#include <astra_camera_msgs/srv/get_camera_info.hpp>

#include "smart_follower_perception/constants.hpp"
#include "smart_follower_perception/frame_sync.hpp"
#include "smart_follower_perception/geometry_utils.hpp"
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
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Image = sensor_msgs::msg::Image;

  PerceptionNode()
  : rclcpp_lifecycle::LifecycleNode("perception_node"),
    diagnostics_(this),
    pipeline_(
      get_logger(),
      *get_clock(),
      params_,
      camera_intrinsics_,
      stats_,
      tracker_,
      lock_manager_,
      yolo_,
      reid_)
  {
    ::smart_follower_perception::declare_parameters(*this, params_);
    sync_intrinsics_diagnostics();
  }

private:
  static CameraIntrinsics camera_info_to_intrinsics(
    const sensor_msgs::msg::CameraInfo & info)
  {
    CameraIntrinsics intrinsics;
    intrinsics.image_width = static_cast<int>(info.width);
    intrinsics.image_height = static_cast<int>(info.height);
    if (info.k.size() >= 9) {
      intrinsics.fx = info.k[0];
      intrinsics.fy = info.k[4];
      intrinsics.cx = info.k[2];
      intrinsics.cy = info.k[5];
    }
    intrinsics.ready = true;
    intrinsics.ready = is_valid_camera_intrinsics(intrinsics);
    return intrinsics;
  }

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

  bool needs_interface_recreation(const PerceptionParams & next) const
  {
    return next.color_topic != params_.color_topic ||
           next.depth_topic != params_.depth_topic ||
           next.person_pose_topic != params_.person_pose_topic ||
           next.follow_command_topic != params_.follow_command_topic;
  }

  bool needs_model_reconfiguration(const PerceptionParams & next) const
  {
    return next.yolo_model_path != params_.yolo_model_path ||
           next.yolo_input_w != params_.yolo_input_w ||
           next.yolo_input_h != params_.yolo_input_h ||
           next.person_class_id != params_.person_class_id ||
           next.yolo_conf_threshold != params_.yolo_conf_threshold ||
           next.yolo_ort.intra_op_num_threads != params_.yolo_ort.intra_op_num_threads ||
           next.yolo_ort.inter_op_num_threads != params_.yolo_ort.inter_op_num_threads ||
           next.yolo_ort.execution_mode_parallel != params_.yolo_ort.execution_mode_parallel ||
           next.reid_model_path != params_.reid_model_path ||
           next.reid_input_w != params_.reid_input_w ||
           next.reid_input_h != params_.reid_input_h ||
           next.reid_ort.intra_op_num_threads != params_.reid_ort.intra_op_num_threads ||
           next.reid_ort.inter_op_num_threads != params_.reid_ort.inter_op_num_threads ||
           next.reid_ort.execution_mode_parallel != params_.reid_ort.execution_mode_parallel;
  }

  bool needs_intrinsics_refresh(const PerceptionParams & next) const
  {
    return !stats_.intrinsics_ready || next.camera.info_service != params_.camera.info_service;
  }

  bool is_active()
  {
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void stop_runtime_processing(bool cancel_timer)
  {
    if (cancel_timer && result_timer_) {
      result_timer_->cancel();
    }
    pipeline_.stop_detection_worker();
    pipeline_.clear_async_state();
  }

  void restart_runtime_processing_if_active(bool was_active)
  {
    if (!was_active) {
      return;
    }
    pipeline_.start_detection_worker();
    if (result_timer_) {
      result_timer_->reset();
    }
  }

  rcl_interfaces::msg::SetParametersResult make_parameter_failure_result(const std::string & reason)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
  }

  rcl_interfaces::msg::SetParametersResult make_parameter_success_result()
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void sync_intrinsics_diagnostics()
  {
    stats_.intrinsics_ready = is_valid_camera_intrinsics(camera_intrinsics_);
    stats_.intrinsics_source = intrinsics_source_;
    stats_.camera_fx = camera_intrinsics_.fx;
    stats_.camera_fy = camera_intrinsics_.fy;
    stats_.camera_cx = camera_intrinsics_.cx;
    stats_.camera_cy = camera_intrinsics_.cy;
  }

  void set_intrinsics(
    const CameraIntrinsics & intrinsics,
    const std::string & source,
    bool log_result)
  {
    camera_intrinsics_ = intrinsics;
    intrinsics_source_ = source;
    sync_intrinsics_diagnostics();

    if (log_result) {
      RCLCPP_INFO(
        get_logger(),
        "[%s] camera intrinsics updated. source=%s ready=%d",
        kRuntimeVersion,
        intrinsics_source_.c_str(),
        stats_.intrinsics_ready ? 1 : 0);
    }
  }

  void log_intrinsics_retry_summary(
    const char * outcome,
    int attempts,
    int max_attempts) const
  {
    RCLCPP_WARN(
      get_logger(),
      "[%s] camera intrinsics service %s: %s after %d/%d attempts",
      kRuntimeVersion,
      outcome,
      params_.camera.info_service.c_str(),
      attempts,
      max_attempts);
  }

  bool try_configure_intrinsics_from_service(bool log_result)
  {
    using GetCameraInfo = astra_camera_msgs::srv::GetCameraInfo;

    if (params_.camera.info_service.empty()) {
      return false;
    }

    constexpr auto kServiceWaitTimeout = std::chrono::milliseconds(1000);
    constexpr auto kRequestTimeout = std::chrono::milliseconds(1500);
    constexpr auto kRetryDelay = std::chrono::milliseconds(350);
    constexpr int kMaxAttempts = 6;

    const auto suffix = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    auto helper = std::make_shared<rclcpp::Node>("perception_intrinsics_client_" + suffix);
    auto client = helper->create_client<GetCameraInfo>(params_.camera.info_service);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(helper);

    bool success = false;
    bool saw_service_unavailable = false;
    bool saw_request_timeout = false;
    bool saw_service_failure = false;
    bool saw_invalid_calibration = false;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
      if (!client->wait_for_service(kServiceWaitTimeout)) {
        saw_service_unavailable = true;
        if (attempt < kMaxAttempts) {
          std::this_thread::sleep_for(kRetryDelay);
        }
        continue;
      }

      auto request = std::make_shared<GetCameraInfo::Request>();
      auto future = client->async_send_request(request);
      const auto status = executor.spin_until_future_complete(future, kRequestTimeout);

      if (status != rclcpp::FutureReturnCode::SUCCESS) {
        saw_request_timeout = true;
        if (attempt < kMaxAttempts) {
          std::this_thread::sleep_for(kRetryDelay);
        }
        continue;
      }

      const auto response = future.get();
      if (!response || !response->success) {
        saw_service_failure = true;
        if (attempt < kMaxAttempts) {
          std::this_thread::sleep_for(kRetryDelay);
        }
        continue;
      }

      const auto intrinsics = camera_info_to_intrinsics(response->info);
      if (!is_valid_camera_intrinsics(intrinsics)) {
        saw_invalid_calibration = true;
        if (attempt < kMaxAttempts) {
          std::this_thread::sleep_for(kRetryDelay);
        }
        continue;
      }

      set_intrinsics(intrinsics, "service", false);
      if (log_result) {
        if (attempt > 1) {
          RCLCPP_INFO(
            get_logger(),
            "[%s] camera intrinsics loaded from service after %d/%d attempts.",
            kRuntimeVersion,
            attempt,
            kMaxAttempts);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "[%s] camera intrinsics loaded from service.",
            kRuntimeVersion);
        }
      }
      success = true;
      break;
    }

    if (!success && log_result) {
      if (saw_service_unavailable) {
        log_intrinsics_retry_summary("unavailable", kMaxAttempts, kMaxAttempts);
      } else if (saw_request_timeout) {
        log_intrinsics_retry_summary("timed out", kMaxAttempts, kMaxAttempts);
      } else if (saw_service_failure) {
        log_intrinsics_retry_summary("returned failure", kMaxAttempts, kMaxAttempts);
      } else if (saw_invalid_calibration) {
        log_intrinsics_retry_summary("returned invalid calibration", kMaxAttempts, kMaxAttempts);
      }
    }

    executor.remove_node(helper);
    return success;
  }

  bool refresh_camera_intrinsics(bool log_result)
  {
    if (try_configure_intrinsics_from_service(log_result)) {
      return true;
    }

    if (log_result) {
      RCLCPP_ERROR(
        get_logger(),
        "[%s] camera intrinsics are required but could not be loaded from service: %s",
        kRuntimeVersion,
        params_.camera.info_service.c_str());
    }
    camera_intrinsics_ = CameraIntrinsics{};
    intrinsics_source_ = "missing";
    sync_intrinsics_diagnostics();
    return false;
  }

  void recreate_interfaces(bool preserve_activation)
  {
    const bool was_active = preserve_activation && person_pub_ && person_pub_->is_activated();
    if (was_active) {
      person_pub_->on_deactivate();
    }

    color_sub_.reset();
    depth_sub_.reset();
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

    if (was_active) {
      person_pub_->on_activate();
    }
  }

  void log_raw_input_status()
  {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "[%s] raw_input color=%zu depth=%zu cache=(%zu,%zu) last_stamp=(%.3f, %.3f)",
      kRuntimeVersion,
      stats_.raw_color_count,
      stats_.raw_depth_count,
      frame_sync_.color_size(),
      frame_sync_.depth_size(),
      PerceptionDiagnostics::stamp_seconds_or_negative(stats_.last_color_msg_stamp),
      PerceptionDiagnostics::stamp_seconds_or_negative(stats_.last_depth_msg_stamp));
  }

  void try_process_cached_frames()
  {
    std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_, std::try_to_lock);
    if (!dispatch_lock.owns_lock()) {
      return;
    }

    FrameSynchronizer::Frame frame;
    while (frame_sync_.pop_next(frame)) {
      pipeline_.enqueue_synchronized_frame(
        SynchronizedFrame{frame.color, frame.depth},
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
    if (!is_active()) {
      return;
    }

    while (pipeline_.consume_ready_result(
      person_pub_,
      [this]() { return this->now(); },
      [this]() { diagnostics_.force_update(); }))
    {
    }
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

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    PerceptionParams candidate = params_;
    for (const auto & parameter : parameters) {
      ::smart_follower_perception::apply_parameter_override(candidate, parameter);
    }
    const bool was_active = is_active();
    const auto previous_params = params_;
    const bool should_recreate_interfaces = needs_interface_recreation(candidate);
    const bool reconfigure_models = needs_model_reconfiguration(candidate);
    const bool refresh_intrinsics = needs_intrinsics_refresh(candidate);

    stop_runtime_processing(was_active);

    params_ = candidate;
    configure_modules_from_params();
    if (reconfigure_models) {
      configure_models();
    }
    if (refresh_intrinsics && !refresh_camera_intrinsics(true)) {
      params_ = previous_params;
      configure_modules_from_params();
      if (reconfigure_models) {
        configure_models();
      }
      if (refresh_intrinsics) {
        refresh_camera_intrinsics(false);
      }
      restart_runtime_processing_if_active(was_active);
      return make_parameter_failure_result("camera intrinsics service unavailable");
    }
    if (should_recreate_interfaces) {
      recreate_interfaces(was_active);
    }

    restart_runtime_processing_if_active(was_active);

    RCLCPP_INFO(
      get_logger(),
      "[%s] perception parameters hot-reloaded, runtime continuing.",
      kRuntimeVersion);

    return make_parameter_success_result();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    ::smart_follower_perception::load_parameters(*this, params_);
    configure_modules_from_params();
    configure_models();
    if (!refresh_camera_intrinsics(true)) {
      return CallbackReturn::FAILURE;
    }
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
      "[%s] perception node configured and running.",
      kRuntimeVersion);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (person_pub_) {
      person_pub_->on_activate();
    }
    pipeline_.clear_async_state();
    restart_runtime_processing_if_active(true);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_runtime_processing(true);
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
    command_sub_.reset();
    person_pub_.reset();
    frame_sync_.reset();
    tracker_.reset();
    lock_manager_.reset();
    stats_.reset();
    camera_intrinsics_ = CameraIntrinsics{};
    intrinsics_source_ = "uninitialized";
    sync_intrinsics_diagnostics();
    return CallbackReturn::SUCCESS;
  }

  PerceptionParams params_;
  PerceptionDiagnostics stats_;
  FrameSynchronizer frame_sync_;
  Tracker tracker_;
  LockManager lock_manager_;
  YoloDetector yolo_;
  ReidExtractor reid_;
  diagnostic_updater::Updater diagnostics_;
  CameraIntrinsics camera_intrinsics_;
  std::string intrinsics_source_{"uninitialized"};
  PerceptionPipeline pipeline_;

  rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr
    person_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
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

  const auto configured_state = node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] perception node failed to configure. current_state=%s returned_state=%s",
      smart_follower_perception::kRuntimeVersion,
      node->get_current_state().label().c_str(),
      configured_state.label().c_str());
    rclcpp::shutdown();
    return 1;
  }

  const auto activated_state = node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activated_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
    node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "[%s] perception node failed to activate. current_state=%s returned_state=%s",
      smart_follower_perception::kRuntimeVersion,
      node->get_current_state().label().c_str(),
      activated_state.label().c_str());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

