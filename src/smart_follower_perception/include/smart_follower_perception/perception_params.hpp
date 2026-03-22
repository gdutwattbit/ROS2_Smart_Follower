#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "smart_follower_perception/runtime.hpp"
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
  OrtRuntimeConfig yolo_ort;
  OrtRuntimeConfig reid_ort;
  CostWeights weights;
};

void declare_parameters(rclcpp_lifecycle::LifecycleNode & node, const PerceptionParams & defaults);
void load_parameters(rclcpp_lifecycle::LifecycleNode & node, PerceptionParams & params);
void apply_parameter_override(PerceptionParams & target, const rclcpp::Parameter & param);

}  // namespace smart_follower_perception
