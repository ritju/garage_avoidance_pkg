#pragma once

#include "garage_road_model/types.hpp"

#include <vector>

namespace garage_road_model
{

struct BoundarySplit
{
  std::vector<Eigen::Vector2d> boundary;
  std::vector<Eigen::Vector2d> start_cap;
  std::vector<Eigen::Vector2d> end_cap;
  std::vector<Eigen::Vector2d> side_a;
  std::vector<Eigen::Vector2d> side_b;
};

// 把闭合边界切成两个端口和两条长边。失败时抛出错误，不硬选。
BoundarySplit splitBoundary(
  const std::vector<Eigen::Vector2d> & boundary,
  const ModelConfig & config);

}  // namespace garage_road_model
