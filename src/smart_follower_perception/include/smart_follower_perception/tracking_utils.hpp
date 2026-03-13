#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include <opencv2/core.hpp>

namespace smart_follower_perception
{

struct Detection
{
  cv::Rect2f bbox;
  float confidence{0.0F};
  float depth_m{0.0F};
  std::array<float, 2048> feature{};
  bool feature_valid{false};
};

inline double bbox_iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  const float w = std::max(0.0F, x2 - x1);
  const float h = std::max(0.0F, y2 - y1);
  const float inter = w * h;
  const float area_a = std::max(0.0F, a.width) * std::max(0.0F, a.height);
  const float area_b = std::max(0.0F, b.width) * std::max(0.0F, b.height);
  const float uni = area_a + area_b - inter;
  if (uni <= 1e-6F) {
    return 0.0;
  }
  return inter / uni;
}

inline double normalized_center_distance(
  const cv::Rect2f & a,
  const cv::Rect2f & b,
  const float norm_w,
  const float norm_h)
{
  const cv::Point2f ca(a.x + a.width * 0.5F, a.y + a.height * 0.5F);
  const cv::Point2f cb(b.x + b.width * 0.5F, b.y + b.height * 0.5F);
  const float dx = (ca.x - cb.x) / std::max(1.0F, norm_w);
  const float dy = (ca.y - cb.y) / std::max(1.0F, norm_h);
  const float d = std::sqrt(dx * dx + dy * dy);
  return std::max(0.0, std::min(1.0, static_cast<double>(d)));
}

inline double cosine_distance(const std::array<float, 2048> & a, const std::array<float, 2048> & b)
{
  double dot = 0.0;
  double na = 0.0;
  double nb = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    dot += static_cast<double>(a[i]) * static_cast<double>(b[i]);
    na += static_cast<double>(a[i]) * static_cast<double>(a[i]);
    nb += static_cast<double>(b[i]) * static_cast<double>(b[i]);
  }
  if (na < 1e-12 || nb < 1e-12) {
    return 1.0;
  }
  const double sim = dot / (std::sqrt(na) * std::sqrt(nb));
  return std::max(0.0, std::min(2.0, 1.0 - sim));
}

inline float percentile(std::vector<float> data, float q)
{
  if (data.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  q = std::max(0.0F, std::min(1.0F, q));
  std::sort(data.begin(), data.end());
  const float idx = q * static_cast<float>(data.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
  if (lo == hi) {
    return data[lo];
  }
  const float t = idx - static_cast<float>(lo);
  return data[lo] * (1.0F - t) + data[hi] * t;
}

}  // namespace smart_follower_perception
