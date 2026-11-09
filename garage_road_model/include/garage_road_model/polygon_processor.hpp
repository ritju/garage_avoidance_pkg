#pragma once

#include "garage_road_model/geometry.hpp"

#include <vector>

namespace garage_road_model
{

// 检查边界点是否构成可用多边形。
std::vector<Eigen::Vector2d> validatePolygon(const std::vector<Eigen::Vector2d> & boundary);

// 闭合边界切出的两个端口和两条长边。
struct BoundarySplit
{
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
