#pragma once

#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "smart_follower_perception/tracker.hpp"

namespace smart_follower_perception
{

struct PerceptionParams
{
  std::string color_topic{"/camera/color/image_raw"};
  std::string depth_topic{"/camera/depth/image_raw"};
  std::string camera_info_topic{"/camera/color/camera_info"};
  std::string person_pose_topic{"person_pose"};
  std::string follow_command_topic{"follow_command"};
  std::string base_frame{"base_footprint"};
  std::string yolo_model_path{"models/yolo26n.onnx"};
  std::string reid_model_path{"models/reid_resnet50_2048.onnx"};

  int process_every_n_frames{3};
  int detect_every_n_frames{1};
  int min_confirm_hits{3};
  int max_miss_frames{10};
  int feature_buffer_size{20};
  int sync_cache_size{6};
  int lock_stable_frames{5};
  float lock_hold_sec{0.6F};
  float lock_switch_sec{2.0F};
  float memory_sec{30.0F};
  float sync_slop{0.04F};
  float low_score_threshold{0.1F};
  float high_score_threshold{0.5F};
  float assignment_threshold{0.7F};
  float second_stage_threshold{0.8F};
  float depth_gate_m{1.0F};
  float depth_norm_m{2.0F};
  float depth_min_m{0.2F};
  float depth_max_m{4.0F};
  float ema_alpha{0.2F};
  float reid_recover_threshold{0.70F};
  float lock_center_roi_ratio{0.6F};
  float lock_target_area_ratio{0.04F};
  int yolo_input_w{640};
  int yolo_input_h{640};
  int reid_input_w{128};
  int reid_input_h{256};
  int person_class_id{0};
  float yolo_conf_threshold{0.25F};
  CostWeights weights;
};

inline void declare_parameters(rclcpp_lifecycle::LifecycleNode & node, const PerceptionParams & defaults)
{
  node.declare_parameter("color_topic", defaults.color_topic);
  node.declare_parameter("depth_topic", defaults.depth_topic);
  node.declare_parameter("camera_info_topic", defaults.camera_info_topic);
  node.declare_parameter("person_pose_topic", defaults.person_pose_topic);
  node.declare_parameter("follow_command_topic", defaults.follow_command_topic);
  node.declare_parameter("base_frame", defaults.base_frame);

  node.declare_parameter("yolo.model_path", defaults.yolo_model_path);
  node.declare_parameter("yolo.input_w", defaults.yolo_input_w);
  node.declare_parameter("yolo.input_h", defaults.yolo_input_h);
  node.declare_parameter("yolo.person_class_id", defaults.person_class_id);
  node.declare_parameter("yolo.conf_threshold", defaults.yolo_conf_threshold);

  node.declare_parameter("reid.model_path", defaults.reid_model_path);
  node.declare_parameter("reid.input_w", defaults.reid_input_w);
  node.declare_parameter("reid.input_h", defaults.reid_input_h);
  node.declare_parameter("reid.ema_alpha", defaults.ema_alpha);
  node.declare_parameter("reid.recover_threshold", defaults.reid_recover_threshold);

  node.declare_parameter("sync_slop", defaults.sync_slop);
  node.declare_parameter("process_every_n_frames", defaults.process_every_n_frames);
  node.declare_parameter("detect_every_n_frames", defaults.detect_every_n_frames);
  node.declare_parameter("min_confirm_hits", defaults.min_confirm_hits);
  node.declare_parameter("max_miss_frames", defaults.max_miss_frames);
  node.declare_parameter("feature_buffer_size", defaults.feature_buffer_size);
  node.declare_parameter("sync_cache_size", defaults.sync_cache_size);
  node.declare_parameter("memory_sec", defaults.memory_sec);

  node.declare_parameter("tracking.low_score_threshold", defaults.low_score_threshold);
  node.declare_parameter("tracking.high_score_threshold", defaults.high_score_threshold);
  node.declare_parameter("tracking.assignment_threshold", defaults.assignment_threshold);
  node.declare_parameter("tracking.second_stage_threshold", defaults.second_stage_threshold);
  node.declare_parameter("tracking.depth_gate_m", defaults.depth_gate_m);
  node.declare_parameter("tracking.depth_norm_m", defaults.depth_norm_m);
  node.declare_parameter("tracking.weights.iou", defaults.weights.w_iou);
  node.declare_parameter("tracking.weights.center", defaults.weights.w_center);
  node.declare_parameter("tracking.weights.depth", defaults.weights.w_depth);
  node.declare_parameter("tracking.weights.appearance", defaults.weights.w_appearance);

  node.declare_parameter("depth.min_m", defaults.depth_min_m);
  node.declare_parameter("depth.max_m", defaults.depth_max_m);

  node.declare_parameter("lock.stable_frames", defaults.lock_stable_frames);
  node.declare_parameter("lock.hold_sec", defaults.lock_hold_sec);
  node.declare_parameter("lock.switch_sec", defaults.lock_switch_sec);
  node.declare_parameter("lock.center_roi_ratio", defaults.lock_center_roi_ratio);
  node.declare_parameter("lock.target_area_ratio", defaults.lock_target_area_ratio);
}

inline void load_parameters(rclcpp_lifecycle::LifecycleNode & node, PerceptionParams & params)
{
  params.color_topic = node.get_parameter("color_topic").as_string();
  params.depth_topic = node.get_parameter("depth_topic").as_string();
  params.camera_info_topic = node.get_parameter("camera_info_topic").as_string();
  params.person_pose_topic = node.get_parameter("person_pose_topic").as_string();
  params.follow_command_topic = node.get_parameter("follow_command_topic").as_string();
  params.base_frame = node.get_parameter("base_frame").as_string();

  params.yolo_model_path = node.get_parameter("yolo.model_path").as_string();
  params.yolo_input_w = node.get_parameter("yolo.input_w").as_int();
  params.yolo_input_h = node.get_parameter("yolo.input_h").as_int();
  params.person_class_id = node.get_parameter("yolo.person_class_id").as_int();
  params.yolo_conf_threshold = node.get_parameter("yolo.conf_threshold").as_double();

  params.reid_model_path = node.get_parameter("reid.model_path").as_string();
  params.reid_input_w = node.get_parameter("reid.input_w").as_int();
  params.reid_input_h = node.get_parameter("reid.input_h").as_int();
  params.ema_alpha = node.get_parameter("reid.ema_alpha").as_double();
  params.reid_recover_threshold = node.get_parameter("reid.recover_threshold").as_double();

  params.sync_slop = node.get_parameter("sync_slop").as_double();
  params.process_every_n_frames = std::max<int>(1, static_cast<int>(node.get_parameter("process_every_n_frames").as_int()));
  params.detect_every_n_frames = std::max<int>(1, static_cast<int>(node.get_parameter("detect_every_n_frames").as_int()));
  params.min_confirm_hits = node.get_parameter("min_confirm_hits").as_int();
  params.max_miss_frames = node.get_parameter("max_miss_frames").as_int();
  params.feature_buffer_size = std::max<int>(1, static_cast<int>(node.get_parameter("feature_buffer_size").as_int()));
  params.sync_cache_size = std::max<int>(3, static_cast<int>(node.get_parameter("sync_cache_size").as_int()));
  params.memory_sec = node.get_parameter("memory_sec").as_double();

  params.low_score_threshold = node.get_parameter("tracking.low_score_threshold").as_double();
  params.high_score_threshold = node.get_parameter("tracking.high_score_threshold").as_double();
  params.assignment_threshold = node.get_parameter("tracking.assignment_threshold").as_double();
  params.second_stage_threshold = node.get_parameter("tracking.second_stage_threshold").as_double();
  params.depth_gate_m = node.get_parameter("tracking.depth_gate_m").as_double();
  params.depth_norm_m = node.get_parameter("tracking.depth_norm_m").as_double();
  params.weights.w_iou = node.get_parameter("tracking.weights.iou").as_double();
  params.weights.w_center = node.get_parameter("tracking.weights.center").as_double();
  params.weights.w_depth = node.get_parameter("tracking.weights.depth").as_double();
  params.weights.w_appearance = node.get_parameter("tracking.weights.appearance").as_double();

  params.depth_min_m = node.get_parameter("depth.min_m").as_double();
  params.depth_max_m = node.get_parameter("depth.max_m").as_double();

  params.lock_stable_frames = std::max<int>(1, static_cast<int>(node.get_parameter("lock.stable_frames").as_int()));
  params.lock_hold_sec = node.get_parameter("lock.hold_sec").as_double();
  params.lock_switch_sec = node.get_parameter("lock.switch_sec").as_double();
  params.lock_center_roi_ratio = node.get_parameter("lock.center_roi_ratio").as_double();
  params.lock_target_area_ratio = node.get_parameter("lock.target_area_ratio").as_double();
}

inline void apply_parameter_override(PerceptionParams & target, const rclcpp::Parameter & param)
{
  const auto & name = param.get_name();
  if (name == "color_topic") target.color_topic = param.as_string();
  else if (name == "depth_topic") target.depth_topic = param.as_string();
  else if (name == "camera_info_topic") target.camera_info_topic = param.as_string();
  else if (name == "person_pose_topic") target.person_pose_topic = param.as_string();
  else if (name == "follow_command_topic") target.follow_command_topic = param.as_string();
  else if (name == "base_frame") target.base_frame = param.as_string();
  else if (name == "yolo.model_path") target.yolo_model_path = param.as_string();
  else if (name == "yolo.input_w") target.yolo_input_w = param.as_int();
  else if (name == "yolo.input_h") target.yolo_input_h = param.as_int();
  else if (name == "yolo.person_class_id") target.person_class_id = param.as_int();
  else if (name == "yolo.conf_threshold") target.yolo_conf_threshold = param.as_double();
  else if (name == "reid.model_path") target.reid_model_path = param.as_string();
  else if (name == "reid.input_w") target.reid_input_w = param.as_int();
  else if (name == "reid.input_h") target.reid_input_h = param.as_int();
  else if (name == "reid.ema_alpha") target.ema_alpha = param.as_double();
  else if (name == "reid.recover_threshold") target.reid_recover_threshold = param.as_double();
  else if (name == "sync_slop") target.sync_slop = param.as_double();
  else if (name == "process_every_n_frames") target.process_every_n_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "detect_every_n_frames") target.detect_every_n_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "min_confirm_hits") target.min_confirm_hits = param.as_int();
  else if (name == "max_miss_frames") target.max_miss_frames = param.as_int();
  else if (name == "feature_buffer_size") target.feature_buffer_size = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "sync_cache_size") target.sync_cache_size = std::max<int>(3, static_cast<int>(param.as_int()));
  else if (name == "memory_sec") target.memory_sec = param.as_double();
  else if (name == "tracking.low_score_threshold") target.low_score_threshold = param.as_double();
  else if (name == "tracking.high_score_threshold") target.high_score_threshold = param.as_double();
  else if (name == "tracking.assignment_threshold") target.assignment_threshold = param.as_double();
  else if (name == "tracking.second_stage_threshold") target.second_stage_threshold = param.as_double();
  else if (name == "tracking.depth_gate_m") target.depth_gate_m = param.as_double();
  else if (name == "tracking.depth_norm_m") target.depth_norm_m = param.as_double();
  else if (name == "tracking.weights.iou") target.weights.w_iou = param.as_double();
  else if (name == "tracking.weights.center") target.weights.w_center = param.as_double();
  else if (name == "tracking.weights.depth") target.weights.w_depth = param.as_double();
  else if (name == "tracking.weights.appearance") target.weights.w_appearance = param.as_double();
  else if (name == "depth.min_m") target.depth_min_m = param.as_double();
  else if (name == "depth.max_m") target.depth_max_m = param.as_double();
  else if (name == "lock.stable_frames") target.lock_stable_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "lock.hold_sec") target.lock_hold_sec = param.as_double();
  else if (name == "lock.switch_sec") target.lock_switch_sec = param.as_double();
  else if (name == "lock.center_roi_ratio") target.lock_center_roi_ratio = param.as_double();
  else if (name == "lock.target_area_ratio") target.lock_target_area_ratio = param.as_double();
}

}  // namespace smart_follower_perception
