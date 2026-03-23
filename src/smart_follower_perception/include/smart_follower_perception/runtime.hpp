#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "smart_follower_perception/constants.hpp"

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace smart_follower_perception
{

std::string resolve_model_path(const std::string & input_path);

struct OrtRuntimeConfig
{
  int intra_op_num_threads{1};
  int inter_op_num_threads{1};
  bool execution_mode_parallel{false};
};

struct YoloRuntimeProfile
{
  double preprocess_ms{0.0};
  double run_ms{0.0};
  double postprocess_ms{0.0};

  void reset()
  {
    preprocess_ms = 0.0;
    run_ms = 0.0;
    postprocess_ms = 0.0;
  }
};

struct ReidRuntimeProfile
{
  double preprocess_ms{0.0};
  double run_ms{0.0};

  void reset()
  {
    preprocess_ms = 0.0;
    run_ms = 0.0;
  }
};

class YoloDetector
{
public:
  struct Result
  {
    cv::Rect2f bbox;
    float conf{0.0F};
  };

  void configure(
    const std::string & model_path,
    int input_w,
    int input_h,
    int person_class_id,
    float conf_threshold,
    const OrtRuntimeConfig & ort_config);
  bool ready() const;
  std::vector<Result> detect(const cv::Mat & bgr);
  const YoloRuntimeProfile & last_profile() const { return last_profile_; }

private:
  static std::vector<Result> nms(const std::vector<Result> & input, float iou_thres);
  void init_runtime();
  void reset_runtime_cache();
  void fill_yolo_input_tensor_from_bgr(const cv::Mat & bgr);

  std::string model_path_;
  int input_w_{640};
  int input_h_{480};
  int person_class_id_{0};
  float conf_threshold_{0.25F};
  OrtRuntimeConfig ort_config_{};
  YoloRuntimeProfile last_profile_{};
  cv::Mat resize_scratch_;
#ifdef HAVE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_tensor_;
  std::array<int64_t, 4> input_shape_{1, 3, input_h_, input_w_};
  std::unique_ptr<Ort::MemoryInfo> mem_info_;
  std::array<const char *, 1> input_names_{nullptr};
  std::array<const char *, 1> output_names_{nullptr};
#endif
};

class ReidExtractor
{
public:
  void configure(
    const std::string & model_path,
    int input_w,
    int input_h,
    const OrtRuntimeConfig & ort_config);
  bool ready() const;
  bool consume_output_dim_error(std::string & msg);
  std::array<float, kFeatureDim> extract(const cv::Mat & bgr, const cv::Rect2f & bbox, bool & valid);
  const ReidRuntimeProfile & last_profile() const { return last_profile_; }

private:
  void init_runtime();
  void reset_runtime_cache();
  void fill_reid_input_tensor_from_bgr(const cv::Mat & bgr);

  std::string model_path_;
  int input_w_{128};
  int input_h_{256};
  bool output_dim_mismatch_{false};
  std::string output_dim_error_msg_;
  OrtRuntimeConfig ort_config_{};
  ReidRuntimeProfile last_profile_{};
  cv::Mat resize_scratch_;
#ifdef HAVE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_tensor_;
  std::array<int64_t, 4> input_shape_{1, 3, input_h_, input_w_};
  std::unique_ptr<Ort::MemoryInfo> mem_info_;
  std::array<const char *, 1> input_names_{nullptr};
  std::array<const char *, 1> output_names_{nullptr};
#endif
};

}  // namespace smart_follower_perception
