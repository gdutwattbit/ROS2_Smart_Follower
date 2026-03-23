#include "smart_follower_perception/perception_params.hpp"

#include <algorithm>

namespace smart_follower_perception
{

void declare_parameters(rclcpp_lifecycle::LifecycleNode & node, const PerceptionParams & defaults)
{
  node.declare_parameter("color_topic", defaults.color_topic);
  node.declare_parameter("person_pose_topic", defaults.person_pose_topic);
  node.declare_parameter("follow_command_topic", defaults.follow_command_topic);
  node.declare_parameter("base_frame", defaults.base_frame);

  node.declare_parameter("yolo.model_path", defaults.yolo_model_path);
  node.declare_parameter("yolo.input_w", defaults.yolo_input_w);
  node.declare_parameter("yolo.input_h", defaults.yolo_input_h);
  node.declare_parameter("yolo.person_class_id", defaults.person_class_id);
  node.declare_parameter("yolo.conf_threshold", defaults.yolo_conf_threshold);
  node.declare_parameter("yolo.ort.intra_op_num_threads", defaults.yolo_ort.intra_op_num_threads);
  node.declare_parameter("yolo.ort.inter_op_num_threads", defaults.yolo_ort.inter_op_num_threads);
  node.declare_parameter(
    "yolo.ort.execution_mode",
    defaults.yolo_ort.execution_mode_parallel ? "parallel" : "sequential");

  node.declare_parameter("reid.model_path", defaults.reid_model_path);
  node.declare_parameter("reid.input_w", defaults.reid_input_w);
  node.declare_parameter("reid.input_h", defaults.reid_input_h);
  node.declare_parameter("reid.ema_alpha", defaults.ema_alpha);
  node.declare_parameter("reid.recover_threshold", defaults.reid_recover_threshold);
  node.declare_parameter("reid.ort.intra_op_num_threads", defaults.reid_ort.intra_op_num_threads);
  node.declare_parameter("reid.ort.inter_op_num_threads", defaults.reid_ort.inter_op_num_threads);
  node.declare_parameter(
    "reid.ort.execution_mode",
    defaults.reid_ort.execution_mode_parallel ? "parallel" : "sequential");

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
  node.declare_parameter("tracking.weights.iou", defaults.weights.w_iou);
  node.declare_parameter("tracking.weights.center", defaults.weights.w_center);
  node.declare_parameter("tracking.weights.appearance", defaults.weights.w_appearance);

  node.declare_parameter("monocular.camera_info_service", defaults.monocular.camera_info_service);
  node.declare_parameter("monocular.person_height_m", defaults.monocular.person_height_m);
  node.declare_parameter("monocular.horizontal_fov_deg", defaults.monocular.horizontal_fov_deg);
  node.declare_parameter("monocular.min_range_m", defaults.monocular.min_range_m);
  node.declare_parameter("monocular.max_range_m", defaults.monocular.max_range_m);
  node.declare_parameter("monocular.camera_height_m", defaults.monocular.camera_height_m);
  node.declare_parameter("monocular.camera_pitch_deg", defaults.monocular.camera_pitch_deg);
  node.declare_parameter("monocular.camera_x_offset_m", defaults.monocular.camera_x_offset_m);
  node.declare_parameter("monocular.camera_y_offset_m", defaults.monocular.camera_y_offset_m);
  node.declare_parameter("monocular.min_downward_angle_deg", defaults.monocular.min_downward_angle_deg);

  node.declare_parameter("lock.stable_frames", defaults.lock_stable_frames);
  node.declare_parameter("lock.hold_sec", defaults.lock_hold_sec);
  node.declare_parameter("lock.switch_sec", defaults.lock_switch_sec);
  node.declare_parameter("lock.center_roi_ratio", defaults.lock_center_roi_ratio);
  node.declare_parameter("lock.target_area_ratio", defaults.lock_target_area_ratio);
}

void load_parameters(rclcpp_lifecycle::LifecycleNode & node, PerceptionParams & params)
{
  params.color_topic = node.get_parameter("color_topic").as_string();
  params.person_pose_topic = node.get_parameter("person_pose_topic").as_string();
  params.follow_command_topic = node.get_parameter("follow_command_topic").as_string();
  params.base_frame = node.get_parameter("base_frame").as_string();

  params.yolo_model_path = node.get_parameter("yolo.model_path").as_string();
  params.yolo_input_w = node.get_parameter("yolo.input_w").as_int();
  params.yolo_input_h = node.get_parameter("yolo.input_h").as_int();
  params.person_class_id = node.get_parameter("yolo.person_class_id").as_int();
  params.yolo_conf_threshold = node.get_parameter("yolo.conf_threshold").as_double();
  params.yolo_ort.intra_op_num_threads = std::max<int>(1, static_cast<int>(node.get_parameter("yolo.ort.intra_op_num_threads").as_int()));
  params.yolo_ort.inter_op_num_threads = std::max<int>(1, static_cast<int>(node.get_parameter("yolo.ort.inter_op_num_threads").as_int()));
  params.yolo_ort.execution_mode_parallel = node.get_parameter("yolo.ort.execution_mode").as_string() == "parallel";

  params.reid_model_path = node.get_parameter("reid.model_path").as_string();
  params.reid_input_w = node.get_parameter("reid.input_w").as_int();
  params.reid_input_h = node.get_parameter("reid.input_h").as_int();
  params.ema_alpha = node.get_parameter("reid.ema_alpha").as_double();
  params.reid_recover_threshold = node.get_parameter("reid.recover_threshold").as_double();
  params.reid_ort.intra_op_num_threads = std::max<int>(1, static_cast<int>(node.get_parameter("reid.ort.intra_op_num_threads").as_int()));
  params.reid_ort.inter_op_num_threads = std::max<int>(1, static_cast<int>(node.get_parameter("reid.ort.inter_op_num_threads").as_int()));
  params.reid_ort.execution_mode_parallel = node.get_parameter("reid.ort.execution_mode").as_string() == "parallel";

  params.process_every_n_frames = std::max<int>(1, static_cast<int>(node.get_parameter("process_every_n_frames").as_int()));
  params.detect_every_n_frames = std::max<int>(1, static_cast<int>(node.get_parameter("detect_every_n_frames").as_int()));
  params.min_confirm_hits = node.get_parameter("min_confirm_hits").as_int();
  params.max_miss_frames = node.get_parameter("max_miss_frames").as_int();
  params.feature_buffer_size = std::max<int>(1, static_cast<int>(node.get_parameter("feature_buffer_size").as_int()));
  params.sync_cache_size = std::max<int>(1, static_cast<int>(node.get_parameter("sync_cache_size").as_int()));
  params.memory_sec = node.get_parameter("memory_sec").as_double();

  params.low_score_threshold = node.get_parameter("tracking.low_score_threshold").as_double();
  params.high_score_threshold = node.get_parameter("tracking.high_score_threshold").as_double();
  params.assignment_threshold = node.get_parameter("tracking.assignment_threshold").as_double();
  params.second_stage_threshold = node.get_parameter("tracking.second_stage_threshold").as_double();
  params.weights.w_iou = node.get_parameter("tracking.weights.iou").as_double();
  params.weights.w_center = node.get_parameter("tracking.weights.center").as_double();
  params.weights.w_appearance = node.get_parameter("tracking.weights.appearance").as_double();

  params.monocular.camera_info_service = node.get_parameter("monocular.camera_info_service").as_string();
  params.monocular.person_height_m = node.get_parameter("monocular.person_height_m").as_double();
  params.monocular.horizontal_fov_deg = node.get_parameter("monocular.horizontal_fov_deg").as_double();
  params.monocular.min_range_m = node.get_parameter("monocular.min_range_m").as_double();
  params.monocular.max_range_m = node.get_parameter("monocular.max_range_m").as_double();
  params.monocular.camera_height_m = std::max<double>(1e-3, node.get_parameter("monocular.camera_height_m").as_double());
  params.monocular.camera_pitch_deg = node.get_parameter("monocular.camera_pitch_deg").as_double();
  params.monocular.camera_x_offset_m = node.get_parameter("monocular.camera_x_offset_m").as_double();
  params.monocular.camera_y_offset_m = node.get_parameter("monocular.camera_y_offset_m").as_double();
  params.monocular.min_downward_angle_deg = std::max<double>(0.0, node.get_parameter("monocular.min_downward_angle_deg").as_double());

  params.lock_stable_frames = std::max<int>(1, static_cast<int>(node.get_parameter("lock.stable_frames").as_int()));
  params.lock_hold_sec = node.get_parameter("lock.hold_sec").as_double();
  params.lock_switch_sec = node.get_parameter("lock.switch_sec").as_double();
  params.lock_center_roi_ratio = node.get_parameter("lock.center_roi_ratio").as_double();
  params.lock_target_area_ratio = node.get_parameter("lock.target_area_ratio").as_double();
}

void apply_parameter_override(PerceptionParams & target, const rclcpp::Parameter & param)
{
  const auto & name = param.get_name();
  if (name == "color_topic") target.color_topic = param.as_string();
  else if (name == "person_pose_topic") target.person_pose_topic = param.as_string();
  else if (name == "follow_command_topic") target.follow_command_topic = param.as_string();
  else if (name == "base_frame") target.base_frame = param.as_string();
  else if (name == "yolo.model_path") target.yolo_model_path = param.as_string();
  else if (name == "yolo.input_w") target.yolo_input_w = param.as_int();
  else if (name == "yolo.input_h") target.yolo_input_h = param.as_int();
  else if (name == "yolo.person_class_id") target.person_class_id = param.as_int();
  else if (name == "yolo.conf_threshold") target.yolo_conf_threshold = param.as_double();
  else if (name == "yolo.ort.intra_op_num_threads") target.yolo_ort.intra_op_num_threads = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "yolo.ort.inter_op_num_threads") target.yolo_ort.inter_op_num_threads = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "yolo.ort.execution_mode") target.yolo_ort.execution_mode_parallel = param.as_string() == "parallel";
  else if (name == "reid.model_path") target.reid_model_path = param.as_string();
  else if (name == "reid.input_w") target.reid_input_w = param.as_int();
  else if (name == "reid.input_h") target.reid_input_h = param.as_int();
  else if (name == "reid.ema_alpha") target.ema_alpha = param.as_double();
  else if (name == "reid.recover_threshold") target.reid_recover_threshold = param.as_double();
  else if (name == "reid.ort.intra_op_num_threads") target.reid_ort.intra_op_num_threads = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "reid.ort.inter_op_num_threads") target.reid_ort.inter_op_num_threads = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "reid.ort.execution_mode") target.reid_ort.execution_mode_parallel = param.as_string() == "parallel";
  else if (name == "process_every_n_frames") target.process_every_n_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "detect_every_n_frames") target.detect_every_n_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "min_confirm_hits") target.min_confirm_hits = param.as_int();
  else if (name == "max_miss_frames") target.max_miss_frames = param.as_int();
  else if (name == "feature_buffer_size") target.feature_buffer_size = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "sync_cache_size") target.sync_cache_size = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "memory_sec") target.memory_sec = param.as_double();
  else if (name == "tracking.low_score_threshold") target.low_score_threshold = param.as_double();
  else if (name == "tracking.high_score_threshold") target.high_score_threshold = param.as_double();
  else if (name == "tracking.assignment_threshold") target.assignment_threshold = param.as_double();
  else if (name == "tracking.second_stage_threshold") target.second_stage_threshold = param.as_double();
  else if (name == "tracking.weights.iou") target.weights.w_iou = param.as_double();
  else if (name == "tracking.weights.center") target.weights.w_center = param.as_double();
  else if (name == "tracking.weights.appearance") target.weights.w_appearance = param.as_double();
  else if (name == "monocular.camera_info_service") target.monocular.camera_info_service = param.as_string();
  else if (name == "monocular.person_height_m") target.monocular.person_height_m = param.as_double();
  else if (name == "monocular.horizontal_fov_deg") target.monocular.horizontal_fov_deg = param.as_double();
  else if (name == "monocular.min_range_m") target.monocular.min_range_m = param.as_double();
  else if (name == "monocular.max_range_m") target.monocular.max_range_m = param.as_double();
  else if (name == "monocular.camera_height_m") target.monocular.camera_height_m = std::max<double>(1e-3, param.as_double());
  else if (name == "monocular.camera_pitch_deg") target.monocular.camera_pitch_deg = param.as_double();
  else if (name == "monocular.camera_x_offset_m") target.monocular.camera_x_offset_m = param.as_double();
  else if (name == "monocular.camera_y_offset_m") target.monocular.camera_y_offset_m = param.as_double();
  else if (name == "monocular.min_downward_angle_deg") target.monocular.min_downward_angle_deg = std::max<double>(0.0, param.as_double());
  else if (name == "lock.stable_frames") target.lock_stable_frames = std::max<int>(1, static_cast<int>(param.as_int()));
  else if (name == "lock.hold_sec") target.lock_hold_sec = param.as_double();
  else if (name == "lock.switch_sec") target.lock_switch_sec = param.as_double();
  else if (name == "lock.center_roi_ratio") target.lock_center_roi_ratio = param.as_double();
  else if (name == "lock.target_area_ratio") target.lock_target_area_ratio = param.as_double();
}

}  // namespace smart_follower_perception
