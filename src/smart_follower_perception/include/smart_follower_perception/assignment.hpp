#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace smart_follower_perception
{

struct AssignmentResult
{
  std::vector<std::pair<int, int>> matches;
  std::vector<int> unmatched_rows;
  std::vector<int> unmatched_cols;
};

inline std::vector<int> hungarian_minimize(const std::vector<std::vector<double>> & cost)
{
  const int n = static_cast<int>(cost.size());
  if (n == 0) {
    return {};
  }
  const int m = static_cast<int>(cost[0].size());
  if (m == 0) {
    return std::vector<int>(n, -1);
  }

  bool transposed = false;
  std::vector<std::vector<double>> matrix = cost;
  int rows = n;
  int cols = m;
  if (rows > cols) {
    transposed = true;
    matrix.assign(cols, std::vector<double>(rows, 0.0));
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        matrix[j][i] = cost[i][j];
      }
    }
    std::swap(rows, cols);
  }

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(rows + 1, 0.0), v(cols + 1, 0.0);
  std::vector<int> p(cols + 1, 0), way(cols + 1, 0);

  for (int i = 1; i <= rows; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(cols + 1, inf);
    std::vector<bool> used(cols + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= cols; ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = matrix[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= cols; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assign(rows, -1);
  for (int j = 1; j <= cols; ++j) {
    if (p[j] > 0 && p[j] <= rows) {
      assign[p[j] - 1] = j - 1;
    }
  }

  if (!transposed) {
    return assign;
  }

  std::vector<int> original_assign(n, -1);
  for (int row_t = 0; row_t < rows; ++row_t) {
    const int col_t = assign[row_t];
    if (col_t >= 0 && col_t < n) {
      original_assign[col_t] = row_t;
    }
  }
  return original_assign;
}

inline AssignmentResult solve_assignment(
  const std::vector<std::vector<double>> & cost,
  const double reject_threshold)
{
  AssignmentResult result;

  const int rows = static_cast<int>(cost.size());
  const int cols = rows > 0 ? static_cast<int>(cost[0].size()) : 0;
  if (rows == 0) {
    result.unmatched_cols.resize(cols);
    for (int j = 0; j < cols; ++j) {
      result.unmatched_cols[j] = j;
    }
    return result;
  }
  if (cols == 0) {
    result.unmatched_rows.resize(rows);
    for (int i = 0; i < rows; ++i) {
      result.unmatched_rows[i] = i;
    }
    return result;
  }

  auto assignment = hungarian_minimize(cost);
  std::vector<bool> row_used(rows, false);
  std::vector<bool> col_used(cols, false);

  for (int i = 0; i < rows; ++i) {
    int j = assignment[i];
    if (j < 0 || j >= cols) {
      continue;
    }
    if (cost[i][j] <= reject_threshold) {
      result.matches.emplace_back(i, j);
      row_used[i] = true;
      col_used[j] = true;
    }
  }

  for (int i = 0; i < rows; ++i) {
    if (!row_used[i]) {
      result.unmatched_rows.push_back(i);
    }
  }
  for (int j = 0; j < cols; ++j) {
    if (!col_used[j]) {
      result.unmatched_cols.push_back(j);
    }
  }

  return result;
}

inline double clamp01(const double x)
{
  return std::max(0.0, std::min(1.0, x));
}

}  // namespace smart_follower_perception
