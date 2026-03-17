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

class YoloDetector
{
public:
  struct Result
  {
    cv::Rect2f bbox;
    float conf{0.0F};
  };

  void configure(const std::string & model_path, int input_w, int input_h, int person_class_id, float conf_threshold);
  bool ready() const;
  std::vector<Result> detect(const cv::Mat & bgr);

private:
  static std::vector<Result> nms(const std::vector<Result> & input, float iou_thres);
  void init_runtime();

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
  void configure(const std::string & model_path, int input_w, int input_h);
  bool ready() const;
  bool consume_output_dim_error(std::string & msg);
  std::array<float, kFeatureDim> extract(const cv::Mat & bgr, const cv::Rect2f & bbox, bool & valid);

private:
  void init_runtime();

  std::string model_path_;
  int input_w_{128};
  int input_h_{256};
  bool output_dim_mismatch_{false};
  std::string output_dim_error_msg_;
#ifdef HAVE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
#endif
};

}  // namespace smart_follower_perception
