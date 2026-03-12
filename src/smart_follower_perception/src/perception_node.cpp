
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smart_follower_msgs/msg/follow_command.hpp>
#include <smart_follower_msgs/msg/person_pose_array.hpp>
#include <smart_follower_msgs/msg/tracked_person.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "smart_follower_perception/assignment.hpp"
#include "smart_follower_perception/tracking_utils.hpp"

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace smart_follower_perception
{

namespace
{
constexpr int kStateDim = 10;
constexpr int kMeasureDim = 4;
constexpr int kFeatureDim = 128;
}  // namespace

struct MemoryEntry
{
  int track_id{-1};
  std::array<float, kFeatureDim> feature{};
  rclcpp::Time expiry;
};

struct Track
{
  int id{-1};
  uint8_t state{smart_follower_msgs::msg::TrackedPerson::TENTATIVE};
  float confidence{0.0F};
  cv::KalmanFilter kf;
  cv::Rect2f bbox;
  float depth_m{0.0F};
  int invalid_depth_frames{0};
  int hit_count{0};
  int miss_count{0};
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  std::array<float, kFeatureDim> ema_feature{};
  bool feature_valid{false};
  std::deque<std::array<float, kFeatureDim>> feature_buffer;

  Track()
  : kf(kStateDim, kMeasureDim, 0, CV_32F)
  {
  }
};

struct CostWeights
{
  double w_iou{0.3};
  double w_center{0.2};
  double w_depth{0.1};
  double w_appearance{0.4};
};

class YoloDetector
{
public:
  struct Result
  {
    cv::Rect2f bbox;
    float conf{0.0F};
  };

  void configure(const std::string & model_path, int input_w, int input_h, int person_class_id, float conf_threshold)
  {
    model_path_ = model_path;
    input_w_ = input_w;
    input_h_ = input_h;
    person_class_id_ = person_class_id;
    conf_threshold_ = conf_threshold;
#ifdef HAVE_ONNXRUNTIME
    init_runtime();
#else
    (void)model_path_;
#endif
  }

  bool ready() const
  {
#ifdef HAVE_ONNXRUNTIME
    return session_ != nullptr;
#else
    return false;
#endif
  }

  std::vector<Result> detect(const cv::Mat & bgr)
  {
#ifdef HAVE_ONNXRUNTIME
    if (!session_) {
      return {};
    }

    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(input_w_, input_h_));
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    std::vector<float> input_tensor(3 * input_h_ * input_w_);
    for (int y = 0; y < input_h_; ++y) {
      for (int x = 0; x < input_w_; ++x) {
        const auto pix = resized.at<cv::Vec3f>(y, x);
        input_tensor[y * input_w_ + x] = pix[0];
        input_tensor[input_h_ * input_w_ + y * input_w_ + x] = pix[1];
        input_tensor[2 * input_h_ * input_w_ + y * input_w_ + x] = pix[2];
      }
    }

    std::array<int64_t, 4> input_shape{1, 3, input_h_, input_w_};
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(mem_info, input_tensor.data(), input_tensor.size(), input_shape.data(), input_shape.size());
    std::vector<const char *> input_names{input_name_.c_str()};
    std::vector<const char *> output_names{output_name_.c_str()};
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names.data(), &input, 1, output_names.data(), 1);
    if (outputs.empty()) {
      return {};
    }

    auto & out = outputs.front();
    auto shape = out.GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() != 3) {
      return {};
    }

    const float scale_x = static_cast<float>(bgr.cols) / static_cast<float>(input_w_);
    const float scale_y = static_cast<float>(bgr.rows) / static_cast<float>(input_h_);
    const float * data = out.GetTensorData<float>();
    std::vector<Result> results;

    const int64_t n = shape[1];
    const int64_t c = shape[2];
    for (int64_t i = 0; i < n; ++i) {
      const float * row = data + i * c;
      if (c < 6) {
        continue;
      }
      float obj = row[4];
      int best_cls = -1;
      float best_prob = 0.0F;
      for (int64_t k = 5; k < c; ++k) {
        if (row[k] > best_prob) {
          best_prob = row[k];
          best_cls = static_cast<int>(k - 5);
        }
      }
      const float conf = obj * best_prob;
      if (best_cls != person_class_id_ || conf < conf_threshold_) {
        continue;
      }
      const float cx = row[0];
      const float cy = row[1];
      const float w = row[2];
      const float h = row[3];
      cv::Rect2f box((cx - 0.5F * w) * scale_x, (cy - 0.5F * h) * scale_y, w * scale_x, h * scale_y);
      box &= cv::Rect2f(0.0F, 0.0F, static_cast<float>(bgr.cols), static_cast<float>(bgr.rows));
      if (box.width > 1.0F && box.height > 1.0F) {
        results.push_back(Result{box, conf});
      }
    }
    return nms(results, 0.45F);
#else
    (void)bgr;
    return {};
#endif
  }

private:
#ifdef HAVE_ONNXRUNTIME
  void init_runtime()
  {
    if (model_path_.empty()) {
      return;
    }
    if (!env_) {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "smart_follower_yolo");
    }
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
  }
#endif

  static std::vector<Result> nms(const std::vector<Result> & input, float iou_thres)
  {
    std::vector<Result> dets = input;
    std::sort(dets.begin(), dets.end(), [](const Result & a, const Result & b) { return a.conf > b.conf; });

    std::vector<Result> keep;
    std::vector<bool> suppressed(dets.size(), false);
    for (std::size_t i = 0; i < dets.size(); ++i) {
      if (suppressed[i]) {
        continue;
      }
      keep.push_back(dets[i]);
      for (std::size_t j = i + 1; j < dets.size(); ++j) {
        if (!suppressed[j] && bbox_iou(dets[i].bbox, dets[j].bbox) > iou_thres) {
          suppressed[j] = true;
        }
      }
    }
    return keep;
  }

  std::string model_path_;
  int input_w_{640};
  int input_h_{640};
  int person_class_id_{0};
  float conf_threshold_{0.25F};
#ifdef HAVE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
#endif
};

class ReidExtractor
{
public:
  void configure(const std::string & model_path, int input_w, int input_h)
  {
    model_path_ = model_path;
    input_w_ = input_w;
    input_h_ = input_h;
#ifdef HAVE_ONNXRUNTIME
    init_runtime();
#else
    (void)model_path_;
#endif
  }

  bool ready() const
  {
#ifdef HAVE_ONNXRUNTIME
    return session_ != nullptr;
#else
    return false;
#endif
  }

  std::array<float, kFeatureDim> extract(const cv::Mat & bgr, const cv::Rect2f & bbox, bool & valid)
  {
    std::array<float, kFeatureDim> feat{};
    valid = false;
#ifdef HAVE_ONNXRUNTIME
    if (!session_ || bbox.width < 2.0F || bbox.height < 2.0F) {
      return feat;
    }

    cv::Rect roi = bbox;
    roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (roi.width <= 2 || roi.height <= 2) {
      return feat;
    }

    cv::Mat crop = bgr(roi).clone();
    cv::Mat rgb;
    cv::cvtColor(crop, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(input_w_, input_h_));
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    std::vector<float> input_tensor(3 * input_h_ * input_w_);
    for (int y = 0; y < input_h_; ++y) {
      for (int x = 0; x < input_w_; ++x) {
        const auto pix = resized.at<cv::Vec3f>(y, x);
        input_tensor[y * input_w_ + x] = pix[0];
        input_tensor[input_h_ * input_w_ + y * input_w_ + x] = pix[1];
        input_tensor[2 * input_h_ * input_w_ + y * input_w_ + x] = pix[2];
      }
    }

    std::array<int64_t, 4> input_shape{1, 3, input_h_, input_w_};
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(mem_info, input_tensor.data(), input_tensor.size(), input_shape.data(), input_shape.size());
    std::vector<const char *> input_names{input_name_.c_str()};
    std::vector<const char *> output_names{output_name_.c_str()};
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names.data(), &input, 1, output_names.data(), 1);
    if (outputs.empty()) {
      return feat;
    }

    const float * out_data = outputs[0].GetTensorData<float>();
    auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int64_t total = 1;
    for (auto d : out_shape) {
      if (d > 0) {
        total *= d;
      }
    }
    if (total < kFeatureDim) {
      return feat;
    }

    double norm = 0.0;
    for (int i = 0; i < kFeatureDim; ++i) {
      feat[i] = out_data[i];
      norm += static_cast<double>(feat[i]) * static_cast<double>(feat[i]);
    }
    norm = std::sqrt(std::max(1e-12, norm));
    for (auto & v : feat) {
      v = static_cast<float>(v / norm);
    }
    valid = true;
#else
    (void)bgr;
    (void)bbox;
#endif
    return feat;
  }

private:
#ifdef HAVE_ONNXRUNTIME
  void init_runtime()
  {
    if (model_path_.empty()) {
      return;
    }
    if (!env_) {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "smart_follower_reid");
    }
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
  }
#endif

  std::string model_path_;
  int input_w_{64};
  int input_h_{128};
#ifdef HAVE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
#endif
};

class PerceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  PerceptionNode()
  : rclcpp_lifecycle::LifecycleNode("perception_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
  }

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;
  using ImageSubscriber = message_filters::Subscriber<Image, rclcpp_lifecycle::LifecycleNode>;
  using CameraInfoSubscriber = message_filters::Subscriber<CameraInfo, rclcpp_lifecycle::LifecycleNode>;

  struct Params
  {
    std::string color_topic{"/camera/color/image_raw"};
    std::string depth_topic{"/camera/depth/image_raw"};
    std::string camera_info_topic{"/camera/color/camera_info"};
    std::string person_pose_topic{"person_pose"};
    std::string follow_command_topic{"follow_command"};
    std::string base_frame{"base_footprint"};
    std::string yolo_model_path{"models/yolo26n.onnx"};
    std::string reid_model_path{"models/reid_mobilenetv2_128.onnx"};

    int detect_every_n_frames{3};
    int min_confirm_hits{3};
    int max_miss_frames{10};
    int feature_buffer_size{20};
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
    float reid_recover_threshold{0.65F};
    float lock_center_roi_ratio{0.6F};
    float lock_target_area_ratio{0.04F};
    int yolo_input_w{640};
    int yolo_input_h{480};
    int reid_input_w{64};
    int reid_input_h{128};
    int person_class_id{0};
    float yolo_conf_threshold{0.25F};
    CostWeights weights;
  } p_;

  rclcpp_lifecycle::LifecyclePublisher<smart_follower_msgs::msg::PersonPoseArray>::SharedPtr person_pub_;
  rclcpp::Subscription<smart_follower_msgs::msg::FollowCommand>::SharedPtr command_sub_;

  std::shared_ptr<ImageSubscriber> color_sub_;
  std::shared_ptr<ImageSubscriber> depth_sub_;
  std::shared_ptr<CameraInfoSubscriber> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::unordered_map<int, Track> tracks_;
  std::deque<MemoryEntry> memory_bank_;
  int next_track_id_{1};
  int frame_counter_{0};

  int lock_id_{-1};
  uint8_t lock_state_{smart_follower_msgs::msg::PersonPoseArray::IDLE};
  bool pending_lock_request_{false};
  int pending_stable_track_{-1};
  int pending_stable_count_{0};

  YoloDetector yolo_;
  ReidExtractor reid_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  diagnostic_updater::Updater diagnostics_{this};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t dropped_sync_frames_{0};
  double last_infer_ms_{0.0};
  std::size_t last_detection_count_{0};

  image_geometry::PinholeCameraModel camera_model_;

  void declare_parameters()
  {
    this->declare_parameter("color_topic", p_.color_topic);
    this->declare_parameter("depth_topic", p_.depth_topic);
    this->declare_parameter("camera_info_topic", p_.camera_info_topic);
    this->declare_parameter("person_pose_topic", p_.person_pose_topic);
    this->declare_parameter("follow_command_topic", p_.follow_command_topic);
    this->declare_parameter("base_frame", p_.base_frame);

    this->declare_parameter("yolo.model_path", p_.yolo_model_path);
    this->declare_parameter("yolo.input_w", p_.yolo_input_w);
    this->declare_parameter("yolo.input_h", p_.yolo_input_h);
    this->declare_parameter("yolo.person_class_id", p_.person_class_id);
    this->declare_parameter("yolo.conf_threshold", p_.yolo_conf_threshold);

    this->declare_parameter("reid.model_path", p_.reid_model_path);
    this->declare_parameter("reid.input_w", p_.reid_input_w);
    this->declare_parameter("reid.input_h", p_.reid_input_h);
    this->declare_parameter("reid.ema_alpha", p_.ema_alpha);
    this->declare_parameter("reid.recover_threshold", p_.reid_recover_threshold);

    this->declare_parameter("sync_slop", p_.sync_slop);
    this->declare_parameter("detect_every_n_frames", p_.detect_every_n_frames);
    this->declare_parameter("min_confirm_hits", p_.min_confirm_hits);
    this->declare_parameter("max_miss_frames", p_.max_miss_frames);
    this->declare_parameter("feature_buffer_size", p_.feature_buffer_size);
    this->declare_parameter("memory_sec", p_.memory_sec);

    this->declare_parameter("tracking.low_score_threshold", p_.low_score_threshold);
    this->declare_parameter("tracking.high_score_threshold", p_.high_score_threshold);
    this->declare_parameter("tracking.assignment_threshold", p_.assignment_threshold);
    this->declare_parameter("tracking.second_stage_threshold", p_.second_stage_threshold);
    this->declare_parameter("tracking.depth_gate_m", p_.depth_gate_m);
    this->declare_parameter("tracking.depth_norm_m", p_.depth_norm_m);
    this->declare_parameter("tracking.weights.iou", p_.weights.w_iou);
    this->declare_parameter("tracking.weights.center", p_.weights.w_center);
    this->declare_parameter("tracking.weights.depth", p_.weights.w_depth);
    this->declare_parameter("tracking.weights.appearance", p_.weights.w_appearance);

    this->declare_parameter("depth.min_m", p_.depth_min_m);
    this->declare_parameter("depth.max_m", p_.depth_max_m);

    this->declare_parameter("lock.stable_frames", p_.lock_stable_frames);
    this->declare_parameter("lock.hold_sec", p_.lock_hold_sec);
    this->declare_parameter("lock.switch_sec", p_.lock_switch_sec);
    this->declare_parameter("lock.center_roi_ratio", p_.lock_center_roi_ratio);
    this->declare_parameter("lock.target_area_ratio", p_.lock_target_area_ratio);
  }

  void load_parameters()
  {
    p_.color_topic = this->get_parameter("color_topic").as_string();
    p_.depth_topic = this->get_parameter("depth_topic").as_string();
    p_.camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    p_.person_pose_topic = this->get_parameter("person_pose_topic").as_string();
    p_.follow_command_topic = this->get_parameter("follow_command_topic").as_string();
    p_.base_frame = this->get_parameter("base_frame").as_string();

    p_.yolo_model_path = this->get_parameter("yolo.model_path").as_string();
    p_.yolo_input_w = this->get_parameter("yolo.input_w").as_int();
    p_.yolo_input_h = this->get_parameter("yolo.input_h").as_int();
    p_.person_class_id = this->get_parameter("yolo.person_class_id").as_int();
    p_.yolo_conf_threshold = this->get_parameter("yolo.conf_threshold").as_double();

    p_.reid_model_path = this->get_parameter("reid.model_path").as_string();
    p_.reid_input_w = this->get_parameter("reid.input_w").as_int();
    p_.reid_input_h = this->get_parameter("reid.input_h").as_int();
    p_.ema_alpha = this->get_parameter("reid.ema_alpha").as_double();
    p_.reid_recover_threshold = this->get_parameter("reid.recover_threshold").as_double();

    p_.sync_slop = this->get_parameter("sync_slop").as_double();
    p_.detect_every_n_frames = std::max<int>(1, static_cast<int>(this->get_parameter("detect_every_n_frames").as_int()));
    p_.min_confirm_hits = this->get_parameter("min_confirm_hits").as_int();
    p_.max_miss_frames = this->get_parameter("max_miss_frames").as_int();
    p_.feature_buffer_size = std::max<int>(1, static_cast<int>(this->get_parameter("feature_buffer_size").as_int()));
    p_.memory_sec = this->get_parameter("memory_sec").as_double();

    p_.low_score_threshold = this->get_parameter("tracking.low_score_threshold").as_double();
    p_.high_score_threshold = this->get_parameter("tracking.high_score_threshold").as_double();
    p_.assignment_threshold = this->get_parameter("tracking.assignment_threshold").as_double();
    p_.second_stage_threshold = this->get_parameter("tracking.second_stage_threshold").as_double();
    p_.depth_gate_m = this->get_parameter("tracking.depth_gate_m").as_double();
    p_.depth_norm_m = this->get_parameter("tracking.depth_norm_m").as_double();
    p_.weights.w_iou = this->get_parameter("tracking.weights.iou").as_double();
    p_.weights.w_center = this->get_parameter("tracking.weights.center").as_double();
    p_.weights.w_depth = this->get_parameter("tracking.weights.depth").as_double();
    p_.weights.w_appearance = this->get_parameter("tracking.weights.appearance").as_double();

    p_.depth_min_m = this->get_parameter("depth.min_m").as_double();
    p_.depth_max_m = this->get_parameter("depth.max_m").as_double();

    p_.lock_stable_frames = std::max<int>(1, static_cast<int>(this->get_parameter("lock.stable_frames").as_int()));
    p_.lock_hold_sec = this->get_parameter("lock.hold_sec").as_double();
    p_.lock_switch_sec = this->get_parameter("lock.switch_sec").as_double();
    p_.lock_center_roi_ratio = this->get_parameter("lock.center_roi_ratio").as_double();
    p_.lock_target_area_ratio = this->get_parameter("lock.target_area_ratio").as_double();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();

    yolo_.configure(p_.yolo_model_path, p_.yolo_input_w, p_.yolo_input_h, p_.person_class_id, p_.yolo_conf_threshold);
    reid_.configure(p_.reid_model_path, p_.reid_input_w, p_.reid_input_h);

    person_pub_ = this->create_publisher<smart_follower_msgs::msg::PersonPoseArray>(p_.person_pose_topic, rclcpp::SystemDefaultsQoS());
    command_sub_ = this->create_subscription<smart_follower_msgs::msg::FollowCommand>(
      p_.follow_command_topic,
      10,
      std::bind(&PerceptionNode::on_follow_command, this, std::placeholders::_1));

    const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    color_sub_ = std::make_shared<ImageSubscriber>(this, p_.color_topic, sensor_qos);
    depth_sub_ = std::make_shared<ImageSubscriber>(this, p_.depth_topic, sensor_qos);
    info_sub_ = std::make_shared<CameraInfoSubscriber>(this, p_.camera_info_topic, sensor_qos);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30), *color_sub_, *depth_sub_, *info_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(p_.sync_slop));
    sync_->registerCallback(std::bind(&PerceptionNode::on_synchronized_frame, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    diagnostics_.setHardwareID("smart_follower_perception");
    diagnostics_.add("perception_status", this, &PerceptionNode::diagnostics_callback);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PerceptionNode::on_parameters_set, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Configured perception node. YOLO ready=%d ReID ready=%d", yolo_.ready(), reid_.ready());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    person_pub_->on_activate();
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
    sync_.reset();
    color_sub_.reset();
    depth_sub_.reset();
    info_sub_.reset();
    command_sub_.reset();
    person_pub_.reset();
    tracks_.clear();
    memory_bank_.clear();
    next_track_id_ = 1;
    return CallbackReturn::SUCCESS;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter> &)
  {
    load_parameters();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void on_follow_command(const smart_follower_msgs::msg::FollowCommand::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    switch (msg->command) {
      case smart_follower_msgs::msg::FollowCommand::LOCK:
        if (msg->target_id >= 0) {
          lock_id_ = msg->target_id;
          lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
          pending_lock_request_ = false;
        } else {
          pending_lock_request_ = true;
          pending_stable_track_ = -1;
          pending_stable_count_ = 0;
        }
        break;
      case smart_follower_msgs::msg::FollowCommand::UNLOCK:
        lock_id_ = -1;
        lock_state_ = smart_follower_msgs::msg::PersonPoseArray::IDLE;
        pending_lock_request_ = false;
        break;
      case smart_follower_msgs::msg::FollowCommand::RESET:
        lock_id_ = -1;
        lock_state_ = smart_follower_msgs::msg::PersonPoseArray::IDLE;
        pending_lock_request_ = false;
        tracks_.clear();
        memory_bank_.clear();
        break;
      case smart_follower_msgs::msg::FollowCommand::ESTOP:
        lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOST;
        break;
      default:
        break;
    }
  }

  static cv::Matx<float, kStateDim, kStateDim> build_transition(float dt)
  {
    cv::Matx<float, kStateDim, kStateDim> F = cv::Matx<float, kStateDim, kStateDim>::eye();
    F(0, 5) = dt;
    F(1, 6) = dt;
    F(2, 7) = dt;
    F(3, 8) = dt;
    F(4, 9) = dt;
    return F;
  }

  Track create_track(const Detection & det, const rclcpp::Time & stamp)
  {
    Track track;
    track.id = next_track_id_++;
    track.state = smart_follower_msgs::msg::TrackedPerson::TENTATIVE;
    track.confidence = det.confidence;
    track.bbox = det.bbox;
    track.depth_m = det.depth_m;
    track.first_seen = stamp;
    track.last_seen = stamp;
    track.hit_count = 1;
    track.kf.transitionMatrix = cv::Mat(build_transition(1.0F));
    track.kf.measurementMatrix = cv::Mat::zeros(kMeasureDim, kStateDim, CV_32F);
    track.kf.measurementMatrix.at<float>(0, 0) = 1.0F;
    track.kf.measurementMatrix.at<float>(1, 1) = 1.0F;
    track.kf.measurementMatrix.at<float>(2, 3) = 1.0F;
    track.kf.measurementMatrix.at<float>(3, 4) = 1.0F;
    cv::setIdentity(track.kf.processNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(track.kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(track.kf.errorCovPost, cv::Scalar::all(1.0));

    track.kf.statePost.at<float>(0, 0) = det.bbox.x + det.bbox.width * 0.5F;
    track.kf.statePost.at<float>(1, 0) = det.bbox.y + det.bbox.height * 0.5F;
    track.kf.statePost.at<float>(2, 0) = det.depth_m;
    track.kf.statePost.at<float>(3, 0) = det.bbox.width;
    track.kf.statePost.at<float>(4, 0) = det.bbox.height;

    if (det.feature_valid) {
      track.feature_valid = true;
      track.ema_feature = det.feature;
      track.feature_buffer.push_back(det.feature);
    }
    return track;
  }

  void predict_track(Track & track, float dt)
  {
    track.kf.transitionMatrix = cv::Mat(build_transition(dt));
    cv::Mat pred = track.kf.predict();
    const float cx = pred.at<float>(0, 0);
    const float cy = pred.at<float>(1, 0);
    const float w = std::max(2.0F, pred.at<float>(3, 0));
    const float h = std::max(2.0F, pred.at<float>(4, 0));
    track.depth_m = pred.at<float>(2, 0);
    track.bbox = cv::Rect2f(cx - w * 0.5F, cy - h * 0.5F, w, h);
  }

  void update_track(Track & track, const Detection & det, const rclcpp::Time & stamp)
  {
    cv::Mat measurement(kMeasureDim, 1, CV_32F);
    measurement.at<float>(0, 0) = det.bbox.x + det.bbox.width * 0.5F;
    measurement.at<float>(1, 0) = det.bbox.y + det.bbox.height * 0.5F;
    measurement.at<float>(2, 0) = det.bbox.width;
    measurement.at<float>(3, 0) = det.bbox.height;
    track.kf.correct(measurement);

    if (det.depth_m >= p_.depth_min_m && det.depth_m <= p_.depth_max_m) {
      track.depth_m = det.depth_m;
      track.kf.statePost.at<float>(2, 0) = det.depth_m;
      track.invalid_depth_frames = 0;
    } else {
      track.invalid_depth_frames += 1;
    }

    track.bbox = det.bbox;
    track.confidence = det.confidence;
    track.hit_count += 1;
    track.miss_count = 0;
    track.last_seen = stamp;
    if (track.hit_count >= p_.min_confirm_hits) {
      track.state = smart_follower_msgs::msg::TrackedPerson::CONFIRMED;
    }

    if (det.feature_valid) {
      if (!track.feature_valid) {
        track.ema_feature = det.feature;
        track.feature_valid = true;
      } else {
        for (int i = 0; i < kFeatureDim; ++i) {
          track.ema_feature[i] = static_cast<float>((1.0 - p_.ema_alpha) * track.ema_feature[i] + p_.ema_alpha * det.feature[i]);
        }
      }
      track.feature_buffer.push_back(det.feature);
      while (static_cast<int>(track.feature_buffer.size()) > p_.feature_buffer_size) {
        track.feature_buffer.pop_front();
      }
    }
  }

  double appearance_cost(const Track & t, const Detection & d) const
  {
    if (!t.feature_valid || !d.feature_valid) {
      return 1.0;
    }
    return std::min(1.0, cosine_distance(t.ema_feature, d.feature));
  }

  double full_cost(const Track & t, const Detection & d, float img_w, float img_h) const
  {
    const double iou_cost = 1.0 - bbox_iou(t.bbox, d.bbox);
    const double center_cost = normalized_center_distance(t.bbox, d.bbox, img_w, img_h);
    double depth_cost = 1.0;
    if (d.depth_m > 0.0F && t.depth_m > 0.0F) {
      const double dd = std::abs(static_cast<double>(d.depth_m) - static_cast<double>(t.depth_m));
      depth_cost = clamp01(dd / std::max(1e-3, static_cast<double>(p_.depth_norm_m)));
      if (dd > p_.depth_gate_m) {
        return 2.0;
      }
    }
    return p_.weights.w_iou * iou_cost + p_.weights.w_center * center_cost +
           p_.weights.w_depth * depth_cost + p_.weights.w_appearance * appearance_cost(t, d);
  }

  double second_stage_cost(const Track & t, const Detection & d) const
  {
    const double iou_cost = 1.0 - bbox_iou(t.bbox, d.bbox);
    return 0.7 * iou_cost + 0.3 * appearance_cost(t, d);
  }

  float sample_depth_m(const cv::Mat & depth, int cx, int cy) const
  {
    if (depth.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const int x0 = std::max(0, cx - 2);
    const int y0 = std::max(0, cy - 2);
    const int x1 = std::min(depth.cols - 1, cx + 2);
    const int y1 = std::min(depth.rows - 1, cy + 2);

    std::vector<float> valid;
    valid.reserve(25);
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        float m = 0.0F;
        if (depth.type() == CV_16UC1) {
          const uint16_t mm = depth.at<uint16_t>(y, x);
          if (mm == 0) {
            continue;
          }
          m = static_cast<float>(mm) * 0.001F;
        } else if (depth.type() == CV_32FC1) {
          m = depth.at<float>(y, x);
        } else {
          continue;
        }

        if (std::isfinite(m) && m >= p_.depth_min_m && m <= p_.depth_max_m) {
          valid.push_back(m);
        }
      }
    }
    if (valid.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    std::nth_element(valid.begin(), valid.begin() + valid.size() / 2, valid.end());
    return valid[valid.size() / 2];
  }

  void update_memory_bank(const rclcpp::Time & now)
  {
    while (!memory_bank_.empty() && memory_bank_.front().expiry < now) {
      memory_bank_.pop_front();
    }
  }

  std::optional<int> try_recover_lock_from_memory(const Detection & det, const rclcpp::Time & now)
  {
    update_memory_bank(now);
    if (!det.feature_valid) {
      return std::nullopt;
    }
    int best_id = -1;
    double best_sim = -1.0;
    for (const auto & entry : memory_bank_) {
      const double sim = 1.0 - cosine_distance(entry.feature, det.feature);
      if (sim > best_sim) {
        best_sim = sim;
        best_id = entry.track_id;
      }
    }
    if (best_id >= 0 && best_sim >= p_.reid_recover_threshold) {
      return best_id;
    }
    return std::nullopt;
  }

  std::optional<geometry_msgs::msg::Point> pixel_to_base_point(const cv::Rect2f & bbox, float depth_m, const std_msgs::msg::Header & header)
  {
    if (!std::isfinite(depth_m) || depth_m < p_.depth_min_m || depth_m > p_.depth_max_m) {
      return std::nullopt;
    }
    const double cx = static_cast<double>(bbox.x + bbox.width * 0.5F);
    const double cy = static_cast<double>(bbox.y + bbox.height * 0.5F);

    const auto ray = camera_model_.projectPixelTo3dRay(cv::Point2d(cx, cy));
    geometry_msgs::msg::PointStamped cam_pt;
    cam_pt.header = header;
    cam_pt.point.x = ray.x * depth_m;
    cam_pt.point.y = ray.y * depth_m;
    cam_pt.point.z = depth_m;

    try {
      const auto tf = tf_buffer_.lookupTransform(p_.base_frame, header.frame_id, header.stamp, tf2::durationFromSec(0.02));
      geometry_msgs::msg::PointStamped base_pt;
      tf2::doTransform(cam_pt, base_pt, tf);
      return base_pt.point;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed (%s <- %s): %s", p_.base_frame.c_str(), header.frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void handle_lock_logic(const cv::Size & image_size)
  {
    if (pending_lock_request_) {
      int best_track = -1;
      double best_score = -1.0;
      const float roi_w = static_cast<float>(image_size.width) * p_.lock_center_roi_ratio;
      const float roi_h = static_cast<float>(image_size.height) * p_.lock_center_roi_ratio;
      const cv::Rect2f roi((image_size.width - roi_w) * 0.5F, (image_size.height - roi_h) * 0.5F, roi_w, roi_h);

      for (auto & [id, t] : tracks_) {
        if (t.state != smart_follower_msgs::msg::TrackedPerson::CONFIRMED) {
          continue;
        }
        cv::Point2f c(t.bbox.x + t.bbox.width * 0.5F, t.bbox.y + t.bbox.height * 0.5F);
        if (!roi.contains(c)) {
          continue;
        }

        const float uc = static_cast<float>(image_size.width) * 0.5F;
        const float vc = static_cast<float>(image_size.height) * 0.5F;
        const float rmax = static_cast<float>(image_size.width) * 0.5F;
        const float d = std::hypot(c.x - uc, c.y - vc);
        const double center_score = 1.0 - std::min(1.0, static_cast<double>(d / std::max(1.0F, rmax)));

        const double area = static_cast<double>(t.bbox.area());
        const double target_area = static_cast<double>(image_size.width * image_size.height) * p_.lock_target_area_ratio;
        const double area_score = std::min(area / target_area, target_area / std::max(1.0, area));
        const double score = 0.5 * center_score + 0.3 * area_score + 0.2 * t.confidence;
        if (score > best_score) {
          best_score = score;
          best_track = id;
        }
      }

      if (best_track >= 0) {
        if (pending_stable_track_ == best_track) {
          pending_stable_count_++;
        } else {
          pending_stable_track_ = best_track;
          pending_stable_count_ = 1;
        }
      } else {
        pending_stable_track_ = -1;
        pending_stable_count_ = 0;
      }

      if (pending_stable_count_ >= p_.lock_stable_frames && pending_stable_track_ >= 0) {
        lock_id_ = pending_stable_track_;
        lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
        pending_lock_request_ = false;
        pending_stable_track_ = -1;
        pending_stable_count_ = 0;
      }
    }

    if (lock_id_ >= 0) {
      auto it = tracks_.find(lock_id_);
      lock_state_ = (it != tracks_.end() && it->second.state == smart_follower_msgs::msg::TrackedPerson::CONFIRMED)
                    ? smart_follower_msgs::msg::PersonPoseArray::LOCKED
                    : smart_follower_msgs::msg::PersonPoseArray::LOST;
    } else {
      lock_state_ = smart_follower_msgs::msg::PersonPoseArray::IDLE;
    }
  }

  void run_tracking(std::vector<Detection> detections, const cv::Size & image_size, const rclcpp::Time & stamp)
  {
    float dt = 0.1F;
    if (last_stamp_.nanoseconds() > 0) {
      dt = std::max(1e-3F, static_cast<float>((stamp - last_stamp_).seconds()));
    }
    last_stamp_ = stamp;

    for (auto & kv : tracks_) {
      predict_track(kv.second, dt);
    }

    std::vector<Detection> high_det;
    std::vector<Detection> low_det;
    for (const auto & d : detections) {
      if (d.confidence >= p_.high_score_threshold) {
        high_det.push_back(d);
      } else if (d.confidence >= p_.low_score_threshold) {
        low_det.push_back(d);
      }
    }

    std::vector<int> track_ids;
    track_ids.reserve(tracks_.size());
    for (const auto & kv : tracks_) {
      track_ids.push_back(kv.first);
    }

    std::vector<std::vector<double>> cost1(track_ids.size(), std::vector<double>(high_det.size(), 2.0));
    for (std::size_t i = 0; i < track_ids.size(); ++i) {
      const auto & t = tracks_.at(track_ids[i]);
      for (std::size_t j = 0; j < high_det.size(); ++j) {
        cost1[i][j] = full_cost(t, high_det[j], image_size.width, image_size.height);
      }
    }

    auto a1 = solve_assignment(cost1, p_.assignment_threshold);
    for (const auto & m : a1.matches) {
      update_track(tracks_.at(track_ids[m.first]), high_det[m.second], stamp);
    }

    std::vector<int> unmatched_track_ids;
    for (int idx : a1.unmatched_rows) {
      unmatched_track_ids.push_back(track_ids[idx]);
    }

    std::vector<std::vector<double>> cost2(unmatched_track_ids.size(), std::vector<double>(low_det.size(), 2.0));
    for (std::size_t i = 0; i < unmatched_track_ids.size(); ++i) {
      const auto & t = tracks_.at(unmatched_track_ids[i]);
      for (std::size_t j = 0; j < low_det.size(); ++j) {
        cost2[i][j] = second_stage_cost(t, low_det[j]);
      }
    }

    auto a2 = solve_assignment(cost2, p_.second_stage_threshold);
    for (const auto & m : a2.matches) {
      update_track(tracks_.at(unmatched_track_ids[m.first]), low_det[m.second], stamp);
    }

    for (int det_idx : a1.unmatched_cols) {
      auto t = create_track(high_det[det_idx], stamp);
      tracks_.insert({t.id, t});
    }

    std::unordered_map<int, bool> updated;
    for (const auto & m : a1.matches) {
      updated[track_ids[m.first]] = true;
    }
    for (const auto & m : a2.matches) {
      updated[unmatched_track_ids[m.first]] = true;
    }

    const rclcpp::Duration memory_ttl = rclcpp::Duration::from_seconds(p_.memory_sec);
    std::vector<int> to_remove;
    for (auto & kv : tracks_) {
      auto & t = kv.second;
      if (updated.find(kv.first) == updated.end()) {
        t.miss_count += 1;
        if (t.miss_count > p_.max_miss_frames) {
          if (t.feature_valid) {
            memory_bank_.push_back(MemoryEntry{kv.first, t.ema_feature, stamp + memory_ttl});
          }
          to_remove.push_back(kv.first);
        } else {
          t.state = smart_follower_msgs::msg::TrackedPerson::LOST;
        }
      }
    }

    for (int id : to_remove) {
      tracks_.erase(id);
    }

    update_memory_bank(stamp);
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

    auto pos = pixel_to_base_point(track.bbox, track.depth_m, header);
    if (pos.has_value()) {
      msg.position = pos.value();
    } else {
      msg.position.x = std::numeric_limits<double>::quiet_NaN();
      msg.position.y = std::numeric_limits<double>::quiet_NaN();
      msg.position.z = std::numeric_limits<double>::quiet_NaN();
    }

    msg.velocity.x = track.kf.statePost.at<float>(5, 0);
    msg.velocity.y = track.kf.statePost.at<float>(6, 0);
    msg.velocity.z = track.kf.statePost.at<float>(7, 0);
    msg.depth_m = track.depth_m;
    if (track.feature_valid) {
      for (int i = 0; i < kFeatureDim; ++i) {
        msg.appearance_feature[i] = track.ema_feature[i];
      }
    }
    msg.last_seen = track.last_seen;
    return msg;
  }

  void on_synchronized_frame(const Image::ConstSharedPtr & color_msg, const Image::ConstSharedPtr & depth_msg, const CameraInfo::ConstSharedPtr & info_msg)
  {
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }
    if (!color_msg || !depth_msg || !info_msg) {
      return;
    }

    if (std::abs((rclcpp::Time(color_msg->header.stamp) - rclcpp::Time(depth_msg->header.stamp)).seconds()) > p_.sync_slop) {
      dropped_sync_frames_++;
      return;
    }

    auto t0 = this->now();
    camera_model_.fromCameraInfo(info_msg);

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

    frame_counter_++;
    std::vector<Detection> detections;
    const bool run_detect = (frame_counter_ % p_.detect_every_n_frames == 0);
    if (run_detect) {
      auto dets = yolo_.detect(color);
      detections.reserve(dets.size());
      for (const auto & det : dets) {
        Detection d;
        d.bbox = det.bbox;
        d.confidence = det.conf;
        const int cx = static_cast<int>(d.bbox.x + d.bbox.width * 0.5F);
        const int cy = static_cast<int>(d.bbox.y + d.bbox.height * 0.5F);
        d.depth_m = sample_depth_m(depth, cx, cy);

        bool feature_valid = false;
        d.feature = reid_.extract(color, d.bbox, feature_valid);
        d.feature_valid = feature_valid;

        if (lock_state_ == smart_follower_msgs::msg::PersonPoseArray::LOST && feature_valid) {
          auto recovered = try_recover_lock_from_memory(d, rclcpp::Time(color_msg->header.stamp));
          if (recovered.has_value()) {
            lock_id_ = recovered.value();
            lock_state_ = smart_follower_msgs::msg::PersonPoseArray::LOCKED;
          }
        }

        detections.push_back(d);
      }
    }

    run_tracking(detections, color.size(), rclcpp::Time(color_msg->header.stamp));
    handle_lock_logic(color.size());

    smart_follower_msgs::msg::PersonPoseArray out;
    out.header = color_msg->header;
    out.header.frame_id = p_.base_frame;
    out.lock_id = lock_id_;
    out.lock_state = lock_state_;
    out.persons.reserve(tracks_.size());
    for (const auto & kv : tracks_) {
      out.persons.push_back(to_msg(kv.second, depth_msg->header));
    }

    if (person_pub_ && person_pub_->is_activated()) {
      person_pub_->publish(out);
    }

    last_detection_count_ = detections.size();
    last_infer_ms_ = (this->now() - t0).seconds() * 1000.0;
    diagnostics_.force_update();
  }

  void diagnostics_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.add("active_tracks", static_cast<int>(tracks_.size()));
    stat.add("last_detection_count", static_cast<int>(last_detection_count_));
    stat.add("dropped_sync_frames", static_cast<int>(dropped_sync_frames_));
    stat.add("last_infer_ms", last_infer_ms_);
    stat.add("lock_id", lock_id_);
    stat.add("lock_state", static_cast<int>(lock_state_));
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Perception healthy");
  }
};

}  // namespace smart_follower_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_follower_perception::PerceptionNode>();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}




