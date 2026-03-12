#include <gtest/gtest.h>

#include <vector>

#include "smart_follower_perception/assignment.hpp"

using smart_follower_perception::solve_assignment;

TEST(Assignment, HandlesEmpty)
{
  std::vector<std::vector<double>> cost;
  auto res = solve_assignment(cost, 0.7);
  EXPECT_TRUE(res.matches.empty());
}

TEST(Assignment, FindsSimplePairs)
{
  std::vector<std::vector<double>> cost{
    {0.1, 0.9, 0.9},
    {0.8, 0.2, 0.9},
    {0.9, 0.8, 0.1}
  };
  auto res = solve_assignment(cost, 0.7);
  ASSERT_EQ(res.matches.size(), 3u);
}

TEST(Assignment, RejectsHighCost)
{
  std::vector<std::vector<double>> cost{{0.8}};
  auto res = solve_assignment(cost, 0.7);
  EXPECT_TRUE(res.matches.empty());
  ASSERT_EQ(res.unmatched_rows.size(), 1u);
  ASSERT_EQ(res.unmatched_cols.size(), 1u);
}
