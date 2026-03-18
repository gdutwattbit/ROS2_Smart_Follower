#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>
#include <smart_follower_msgs/msg/tracked_person.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "smart_follower_perception/constants.hpp"
#include "smart_follower_perception/frame_sync.hpp"
#include "smart_follower_perception/geometry_utils.hpp"
#include "smart_follower_perception/lock_manager.hpp"
#include "smart_follower_perception/perception_diagnostics.hpp"
#include "smart_follower_perception/perception_params.hpp"
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
    diagnostics_(this)
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
      params_.yolo_conf_threshold);
    reid_.configure(params_.reid_model_path, params_.reid_input_w, params_.reid_input_h);
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
    std::unique_lock<std::mutex> process_lock(process_mutex_, std::try_to_lock);
    if (!process_lock.owns_lock()) {
      return;
    }

    FrameSynchronizer::Triplet triplet;
    while (frame_sync_.pop_next(triplet)) {
      process_synchronized_frame(triplet.color, triplet.depth, triplet.info);
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

  smart_follower_msgs::msg::TrackedPerson to_msg(const Track & track, const std_msgs::msg::Header & header)
  {
    smart_follower_msgs::msg::TrackedPerson msg;
    msg.track_id = track.id;
    msg.track_state = track.state;
    msg.confidence = track.confidence;
    msg.bbox.x_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.x));
    msg.bbox.y_offset = static_cast<uint32_t>(std::max(0.0F, track.bbox.y));
    msg.bbox.width = static_cast<uint32_t>(std::max(0.0F, track.bbox.width));
    msg.bbox.height = static_cast<uint32_t>(std::max(0.0F, track.bbox.height));

    auto position = pixel_to_base_point(
      track.bbox,
      track.depth_m,
      header,
      camera_model_,
      tf_buffer_,
      params_.base_frame,
      params_.depth_min_m,
      params_.depth_max_m,
      get_logger(),
      *get_clock());
    if (position.has_value()) {
      msg.position = *position;
    } else {
      msg.position.x = std::numeric_limits<double>::quiet_NaN();
      msg.position.y = std::numeric_limits<double>::quiet_NaN();
      msg.position.z = std::numeric_limits<double>::quiet_NaN();
    }

    msg.velocity.x = 0.0;
    msg.velocity.y = 0.0;
    msg.velocity.z = 0.0;
    msg.depth_m = track.depth_m;
    if (track.feature_valid) {
      for (int i = 0; i < kFeatureDim; ++i) {
        msg.appearance_feature[i] = track.ema_feature[i];
      }
    }
    msg.last_seen = track.last_seen;
    return msg;
  }

  void process_synchronized_frame(
    const Image::SharedPtr & color_msg,
    const Image::SharedPtr & depth_msg,
    const CameraInfo::SharedPtr & info_msg)
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }
    if (!color_msg || !depth_msg || !info_msg) {
      return;
    }

    const rclcpp::Time stamp(color_msg->header.stamp);
    if (std::abs((stamp - rclcpp::Time(depth_msg->header.stamp)).seconds()) > params_.sync_slop) {
      stats_.dropped_sync_frames_extra += 1;
      return;
    }

    stats_.synced_callback_count += 1;
    stats_.synced_frame_counter += 1;
    const bool run_pipeline = (stats_.synced_frame_counter % params_.process_every_n_frames == 0);
    if (!run_pipeline) {
      stats_.skipped_synced_frame_count += 1;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[%s] sync callback synced=%zu synced_frame=%d processed=%d run_pipeline=0",
        kRuntimeVersion,
        stats_.synced_callback_count,
        stats_.synced_frame_counter,
        stats_.processed_frame_counter);
      return;
    }

    const auto t0 = this->now();
    camera_model_.fromCameraInfo(*info_msg);

    cv::Mat color;
    cv::Mat depth;
    try {
      color = cv_bridge::toCvShare(color_msg, "bgr8")->image;
      if (depth_msg->encoding == "16UC1" || depth_msg->encoding == "32FC1") {
        depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
      } else {
        depth = cv_bridge::toCvShare(depth_msg)->image;
      }
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge error: %s", ex.what());
      return;
    }

    stats_.processed_frame_counter += 1;
    const bool run_detect = (stats_.processed_frame_counter % params_.detect_every_n_frames == 0);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "[%s] sync callback synced=%zu synced_frame=%d processed=%d run_pipeline=1 run_detect=%d",
      kRuntimeVersion,
      stats_.synced_callback_count,
      stats_.synced_frame_counter,
      stats_.processed_frame_counter,
      run_detect ? 1 : 0);

    std::vector<Detection> detections;
    if (run_detect) {
      auto detector_results = yolo_.detect(color);
      detections.reserve(detector_results.size());
      for (const auto & det : detector_results) {
        Detection detection;
        detection.bbox = det.bbox;
        detection.confidence = det.conf;
        const int cx = static_cast<int>(detection.bbox.x + detection.bbox.width * 0.5F);
        const int cy = static_cast<int>(detection.bbox.y + detection.bbox.height * 0.5F);
        detection.depth_m = sample_depth_m(depth, cx, cy, params_.depth_min_m, params_.depth_max_m);

        bool feature_valid = false;
        detection.feature = reid_.extract(color, detection.bbox, feature_valid);
        detection.feature_valid = feature_valid;

        if (
          lock_manager_.lock_state() == smart_follower_msgs::msg::PersonPoseArray::LOST &&
          feature_valid)
        {
          auto recovered = tracker_.try_recover_lock_from_memory(
            detection,
            stamp,
            lock_manager_.lock_id());
          if (recovered.has_value()) {
            detection.recovered_track_id = *recovered;
          }
        }

        detections.push_back(detection);
      }
    }

    std::string reid_dim_error;
    if (reid_.consume_output_dim_error(reid_dim_error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reid_dim_error.c_str());
    }

    auto track_result = tracker_.run_tracking(detections, color.size(), stamp, run_detect);
    if (track_result.recovered_track_id.has_value()) {
      lock_manager_.set_lock_id(*track_result.recovered_track_id);
    }
    lock_manager_.update(tracker_.tracks(), color.size(), stamp);

    smart_follower_msgs::msg::PersonPoseArray out;
    out.header = color_msg->header;
    out.header.frame_id = params_.base_frame;
    out.lock_id = lock_manager_.lock_id();
    out.lock_state = lock_manager_.lock_state();
    out.persons.reserve(tracker_.tracks().size());
    for (const auto & kv : tracker_.tracks()) {
      out.persons.push_back(to_msg(kv.second, depth_msg->header));
    }

    if (person_pub_ && person_pub_->is_activated()) {
      person_pub_->publish(out);
      stats_.person_pose_publish_count += 1;
      stats_.last_person_pose_publish_stamp = this->now();
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[%s] published person_pose publish_count=%zu persons=%zu lock_id=%d lock_state=%u detections=%zu",
        kRuntimeVersion,
        stats_.person_pose_publish_count,
        out.persons.size(),
        out.lock_id,
        out.lock_state,
        detections.size());
    }

    stats_.last_detection_count = detections.size();
    stats_.last_infer_ms = (this->now() - t0).seconds() * 1000.0;
    diagnostics_.force_update();
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
    params_ = candidate;
    configure_modules_from_params();
    configure_models();
    recreate_interfaces(this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    RCLCPP_INFO(
      get_logger(),
      "[%s] parameters hot-reloaded: color=%s depth=%s info=%s person_pose=%s yolo=%s reid=%s sync_slop=%.3f",
      kRuntimeVersion,
      params_.color_topic.c_str(),
      params_.depth_topic.c_str(),
      params_.camera_info_topic.c_str(),
      params_.person_pose_topic.c_str(),
      params_.yolo_model_path.c_str(),
      params_.reid_model_path.c_str(),
      params_.sync_slop);

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
      "[%s] Configured perception node. YOLO ready=%d ReID ready=%d yolo_input=%dx%d reid_input=%dx%d process_every_n_frames=%d detect_every_n_frames=%d",
      kRuntimeVersion,
      yolo_.ready(),
      reid_.ready(),
      params_.yolo_input_w,
      params_.yolo_input_h,
      params_.reid_input_w,
      params_.reid_input_h,
      params_.process_every_n_frames,
      params_.detect_every_n_frames);
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
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (person_pub_) {
      person_pub_->on_deactivate();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
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

  rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr person_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  bool diagnostics_registered_{false};
  std::mutex process_mutex_;
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


