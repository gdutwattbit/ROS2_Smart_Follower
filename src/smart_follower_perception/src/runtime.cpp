#include "smart_follower_perception/runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#include <opencv2/imgproc.hpp>

#include "smart_follower_perception/tracking_utils.hpp"

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace smart_follower_perception
{

namespace
{
namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;

#ifdef HAVE_ONNXRUNTIME
void apply_ort_runtime_config(Ort::SessionOptions & session_options, const OrtRuntimeConfig & config)
{
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_options.SetExecutionMode(
    config.execution_mode_parallel ? ExecutionMode::ORT_PARALLEL : ExecutionMode::ORT_SEQUENTIAL);
  session_options.SetIntraOpNumThreads(std::max(1, config.intra_op_num_threads));
  session_options.SetInterOpNumThreads(std::max(1, config.inter_op_num_threads));
}
#endif

double elapsed_ms(const SteadyClock::time_point & begin, const SteadyClock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

void fill_rgb_chw_from_bgr_u8(const cv::Mat & bgr, float * dst, bool normalize_255)
{
  const int width = bgr.cols;
  const int height = bgr.rows;
  const int plane_stride = width * height;
  const float scale = normalize_255 ? (1.0F / 255.0F) : 1.0F;

  float * r_plane = dst;
  float * g_plane = dst + plane_stride;
  float * b_plane = dst + plane_stride * 2;

  for (int y = 0; y < height; ++y) {
    const auto * row = bgr.ptr<cv::Vec3b>(y);
    int offset = y * width;
    for (int x = 0; x < width; ++x) {
      const auto & pix = row[x];
      const int index = offset + x;
      r_plane[index] = static_cast<float>(pix[2]) * scale;
      g_plane[index] = static_cast<float>(pix[1]) * scale;
      b_plane[index] = static_cast<float>(pix[0]) * scale;
    }
  }
}

void fill_normalized_rgb_chw_from_bgr_u8(
  const cv::Mat & bgr,
  float * dst,
  const std::array<float, 3> & mean,
  const std::array<float, 3> & std)
{
  const int width = bgr.cols;
  const int height = bgr.rows;
  const int plane_stride = width * height;
  constexpr float kScale = 1.0F / 255.0F;

  float * r_plane = dst;
  float * g_plane = dst + plane_stride;
  float * b_plane = dst + plane_stride * 2;

  for (int y = 0; y < height; ++y) {
    const auto * row = bgr.ptr<cv::Vec3b>(y);
    int offset = y * width;
    for (int x = 0; x < width; ++x) {
      const auto & pix = row[x];
      const int index = offset + x;
      const float r = static_cast<float>(pix[2]) * kScale;
      const float g = static_cast<float>(pix[1]) * kScale;
      const float b = static_cast<float>(pix[0]) * kScale;
      r_plane[index] = (r - mean[0]) / std[0];
      g_plane[index] = (g - mean[1]) / std[1];
      b_plane[index] = (b - mean[2]) / std[2];
    }
  }
}

}  // namespace

std::string resolve_model_path(const std::string & input_path)
{
  if (input_path.empty()) {
    return input_path;
  }

  fs::path relative_path(input_path);
  std::error_code ec;
  auto try_candidate = [&](const fs::path & base) -> std::string {
    if (base.empty()) {
      return {};
    }
    fs::path candidate = base / relative_path;
    if (fs::exists(candidate, ec)) {
      return fs::weakly_canonical(candidate, ec).string();
    }
    return {};
  };

  if (relative_path.is_absolute() && fs::exists(relative_path, ec)) {
    return fs::weakly_canonical(relative_path, ec).string();
  }

  std::vector<fs::path> roots;
  fs::path cwd = fs::current_path(ec);
  if (!ec) {
    roots.push_back(cwd);
    for (fs::path cur = cwd; !cur.empty() && cur != cur.root_path(); cur = cur.parent_path()) {
      roots.push_back(cur);
    }
  }

  if (const char * prefix = std::getenv("COLCON_CURRENT_PREFIX")) {
    fs::path prefix_path(prefix);
    roots.push_back(prefix_path);
    roots.push_back(prefix_path.parent_path());
    roots.push_back(prefix_path.parent_path().parent_path());
  }

#ifdef __linux__
  fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (!ec && !exe.empty()) {
    exe = exe.parent_path();
    roots.push_back(exe);
    for (fs::path cur = exe; !cur.empty() && cur != cur.root_path(); cur = cur.parent_path()) {
      roots.push_back(cur);
    }
  }
#endif

  for (const auto & root : roots) {
    if (auto resolved = try_candidate(root); !resolved.empty()) {
      return resolved;
    }
  }

  return input_path;
}

void YoloDetector::configure(
  const std::string & model_path,
  int input_w,
  int input_h,
  int person_class_id,
  float conf_threshold,
  const OrtRuntimeConfig & ort_config)
{
  model_path_ = model_path;
  input_w_ = input_w;
  input_h_ = input_h;
  person_class_id_ = person_class_id;
  conf_threshold_ = conf_threshold;
  ort_config_ = ort_config;
  reset_runtime_cache();
#ifdef HAVE_ONNXRUNTIME
  session_.reset();
  try {
    init_runtime();
  } catch (const Ort::Exception & ex) {
    std::fprintf(stderr, "[smart_follower][yolo] load model failed (%s): %s\n", model_path_.c_str(), ex.what());
    session_.reset();
    reset_runtime_cache();
  }
#else
  (void)model_path_;
#endif
}

bool YoloDetector::ready() const
{
#ifdef HAVE_ONNXRUNTIME
  return session_ != nullptr;
#else
  return false;
#endif
}

std::vector<YoloDetector::Result> YoloDetector::detect(const cv::Mat & bgr)
{
  last_profile_.reset();
#ifdef HAVE_ONNXRUNTIME
  if (!session_ || !mem_info_) {
    return {};
  }

  const auto preprocess_begin = SteadyClock::now();
  const cv::Mat * input_image = &bgr;
  if (bgr.cols != input_w_ || bgr.rows != input_h_) {
    cv::resize(bgr, resize_scratch_, cv::Size(input_w_, input_h_));
    input_image = &resize_scratch_;
  }
  fill_yolo_input_tensor_from_bgr(*input_image);
  const auto preprocess_end = SteadyClock::now();
  last_profile_.preprocess_ms = elapsed_ms(preprocess_begin, preprocess_end);

  Ort::Value input = Ort::Value::CreateTensor<float>(
    *mem_info_,
    input_tensor_.data(),
    input_tensor_.size(),
    input_shape_.data(),
    input_shape_.size());

  std::vector<Ort::Value> outputs;
  const auto run_begin = SteadyClock::now();
  try {
    outputs = session_->Run(
      Ort::RunOptions{nullptr}, input_names_.data(), &input, 1, output_names_.data(), 1);
  } catch (const Ort::Exception & ex) {
    std::fprintf(stderr, "[smart_follower][yolo] ONNXRuntime inference failed: %s\n", ex.what());
    last_profile_.run_ms = elapsed_ms(run_begin, SteadyClock::now());
    return {};
  }
  const auto run_end = SteadyClock::now();
  last_profile_.run_ms = elapsed_ms(run_begin, run_end);
  if (outputs.empty()) {
    return {};
  }

  const auto postprocess_begin = SteadyClock::now();
  auto & out = outputs.front();
  auto shape = out.GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() != 3) {
    last_profile_.postprocess_ms = elapsed_ms(postprocess_begin, SteadyClock::now());
    return {};
  }

  const float scale_x = static_cast<float>(bgr.cols) / static_cast<float>(input_w_);
  const float scale_y = static_cast<float>(bgr.rows) / static_cast<float>(input_h_);
  const float * data = out.GetTensorData<float>();
  std::vector<Result> results;

  const int64_t n = shape[1];
  const int64_t c = shape[2];
  const bool is_end2end_output = (c == 6);
  for (int64_t i = 0; i < n; ++i) {
    const float * row = data + i * c;
    if (c < 6) {
      continue;
    }

    cv::Rect2f box;
    float conf = 0.0F;
    int best_cls = -1;

    if (is_end2end_output) {
      const float x1 = row[0] * scale_x;
      const float y1 = row[1] * scale_y;
      const float x2 = row[2] * scale_x;
      const float y2 = row[3] * scale_y;
      conf = row[4];
      best_cls = static_cast<int>(std::round(row[5]));
      box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
    } else {
      const float obj = row[4];
      float best_prob = 0.0F;
      for (int64_t k = 5; k < c; ++k) {
        if (row[k] > best_prob) {
          best_prob = row[k];
          best_cls = static_cast<int>(k - 5);
        }
      }
      conf = obj * best_prob;
      const float cx = row[0];
      const float cy = row[1];
      const float w = row[2];
      const float h = row[3];
      box = cv::Rect2f((cx - 0.5F * w) * scale_x, (cy - 0.5F * h) * scale_y, w * scale_x, h * scale_y);
    }

    if (best_cls != person_class_id_ || conf < conf_threshold_) {
      continue;
    }

    box &= cv::Rect2f(0.0F, 0.0F, static_cast<float>(bgr.cols), static_cast<float>(bgr.rows));
    if (box.width > 1.0F && box.height > 1.0F) {
      results.push_back(Result{box, conf});
    }
  }
  auto filtered = nms(results, 0.45F);
  last_profile_.postprocess_ms = elapsed_ms(postprocess_begin, SteadyClock::now());
  return filtered;
#else
  (void)bgr;
  return {};
#endif
}

void YoloDetector::init_runtime()
{
#ifdef HAVE_ONNXRUNTIME
  if (model_path_.empty()) {
    return;
  }
  if (!env_) {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "smart_follower_yolo");
  }
  session_options_ = Ort::SessionOptions{};
  apply_ort_runtime_config(session_options_, ort_config_);
  session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  input_shape_ = {1, 3, static_cast<int64_t>(input_h_), static_cast<int64_t>(input_w_)};
  input_tensor_.assign(static_cast<std::size_t>(3) * static_cast<std::size_t>(input_h_) *
      static_cast<std::size_t>(input_w_), 0.0F);
  mem_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
  input_names_ = {input_name_.c_str()};
  output_names_ = {output_name_.c_str()};
#endif
}

void YoloDetector::reset_runtime_cache()
{
  last_profile_.reset();
  resize_scratch_.release();
#ifdef HAVE_ONNXRUNTIME
  input_tensor_.clear();
  input_shape_ = {1, 3, static_cast<int64_t>(input_h_), static_cast<int64_t>(input_w_)};
  mem_info_.reset();
  input_names_ = {nullptr};
  output_names_ = {nullptr};
#endif
}

void YoloDetector::fill_yolo_input_tensor_from_bgr(const cv::Mat & bgr)
{
#ifdef HAVE_ONNXRUNTIME
  fill_rgb_chw_from_bgr_u8(bgr, input_tensor_.data(), true);
#else
  (void)bgr;
#endif
}

std::vector<YoloDetector::Result> YoloDetector::nms(const std::vector<Result> & input, float iou_thres)
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

void ReidExtractor::configure(
  const std::string & model_path,
  int input_w,
  int input_h,
  const OrtRuntimeConfig & ort_config)
{
  model_path_ = model_path;
  input_w_ = input_w;
  input_h_ = input_h;
  ort_config_ = ort_config;
  output_dim_mismatch_ = false;
  output_dim_error_msg_.clear();
  reset_runtime_cache();
#ifdef HAVE_ONNXRUNTIME
  session_.reset();
  try {
    init_runtime();
  } catch (const Ort::Exception & ex) {
    std::fprintf(stderr, "[smart_follower][reid] load model failed (%s): %s\n", model_path_.c_str(), ex.what());
    session_.reset();
    reset_runtime_cache();
  }
#else
  (void)model_path_;
#endif
}

bool ReidExtractor::ready() const
{
#ifdef HAVE_ONNXRUNTIME
  return session_ != nullptr;
#else
  return false;
#endif
}

bool ReidExtractor::consume_output_dim_error(std::string & msg)
{
  if (!output_dim_mismatch_) {
    return false;
  }
  msg = output_dim_error_msg_;
  output_dim_mismatch_ = false;
  return true;
}

std::array<float, kFeatureDim> ReidExtractor::extract(const cv::Mat & bgr, const cv::Rect2f & bbox, bool & valid)
{
  std::array<float, kFeatureDim> feat{};
  valid = false;
  last_profile_.reset();
#ifdef HAVE_ONNXRUNTIME
  if (!session_ || !mem_info_ || bbox.width < 2.0F || bbox.height < 2.0F) {
    return feat;
  }

  cv::Rect roi = bbox;
  roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (roi.width <= 2 || roi.height <= 2) {
    return feat;
  }

  const auto preprocess_begin = SteadyClock::now();
  const cv::Mat crop = bgr(roi);
  const cv::Mat * input_image = &crop;
  if (crop.cols != input_w_ || crop.rows != input_h_) {
    cv::resize(crop, resize_scratch_, cv::Size(input_w_, input_h_));
    input_image = &resize_scratch_;
  }
  fill_reid_input_tensor_from_bgr(*input_image);
  const auto preprocess_end = SteadyClock::now();
  last_profile_.preprocess_ms = elapsed_ms(preprocess_begin, preprocess_end);

  auto input = Ort::Value::CreateTensor<float>(
    *mem_info_,
    input_tensor_.data(),
    input_tensor_.size(),
    input_shape_.data(),
    input_shape_.size());

  std::vector<Ort::Value> outputs;
  const auto run_begin = SteadyClock::now();
  try {
    outputs = session_->Run(
      Ort::RunOptions{nullptr},
      input_names_.data(),
      &input,
      1,
      output_names_.data(),
      1);
  } catch (const Ort::Exception & ex) {
    std::fprintf(stderr, "[smart_follower][reid] ONNXRuntime inference failed: %s\n", ex.what());
    last_profile_.run_ms = elapsed_ms(run_begin, SteadyClock::now());
    return feat;
  }
  const auto run_end = SteadyClock::now();
  last_profile_.run_ms = elapsed_ms(run_begin, run_end);
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
  if (total <= 0) {
    output_dim_mismatch_ = true;
    output_dim_error_msg_ = "ReID output dim invalid: " + std::to_string(total);
    return feat;
  }
  if (total > kFeatureDim) {
    output_dim_mismatch_ = true;
    output_dim_error_msg_ = "ReID output dim mismatch, expected <= " +
      std::to_string(kFeatureDim) + ", got " + std::to_string(total);
    return feat;
  }
  if (total != kFeatureDim) {
    output_dim_mismatch_ = true;
    output_dim_error_msg_ = "ReID output dim " + std::to_string(total) +
      " padded to " + std::to_string(kFeatureDim) + " for benchmarking compatibility";
  }

  double norm = 0.0;
  for (int64_t i = 0; i < total; ++i) {
    feat[static_cast<std::size_t>(i)] = out_data[i];
    norm += static_cast<double>(feat[static_cast<std::size_t>(i)]) *
      static_cast<double>(feat[static_cast<std::size_t>(i)]);
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

void ReidExtractor::init_runtime()
{
#ifdef HAVE_ONNXRUNTIME
  if (model_path_.empty()) {
    return;
  }
  if (!env_) {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "smart_follower_reid");
  }
  session_options_ = Ort::SessionOptions{};
  apply_ort_runtime_config(session_options_, ort_config_);
  session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  input_shape_ = {1, 3, static_cast<int64_t>(input_h_), static_cast<int64_t>(input_w_)};
  input_tensor_.assign(static_cast<std::size_t>(3) * static_cast<std::size_t>(input_h_) *
      static_cast<std::size_t>(input_w_), 0.0F);
  mem_info_ = std::make_unique<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
  input_names_ = {input_name_.c_str()};
  output_names_ = {output_name_.c_str()};
#endif
}

void ReidExtractor::reset_runtime_cache()
{
  last_profile_.reset();
  resize_scratch_.release();
#ifdef HAVE_ONNXRUNTIME
  input_tensor_.clear();
  input_shape_ = {1, 3, static_cast<int64_t>(input_h_), static_cast<int64_t>(input_w_)};
  mem_info_.reset();
  input_names_ = {nullptr};
  output_names_ = {nullptr};
#endif
}

void ReidExtractor::fill_reid_input_tensor_from_bgr(const cv::Mat & bgr)
{
#ifdef HAVE_ONNXRUNTIME
  constexpr std::array<float, 3> kMean{0.485F, 0.456F, 0.406F};
  constexpr std::array<float, 3> kStd{0.229F, 0.224F, 0.225F};
  fill_normalized_rgb_chw_from_bgr_u8(bgr, input_tensor_.data(), kMean, kStd);
#else
  (void)bgr;
#endif
}

}  // namespace smart_follower_perception
