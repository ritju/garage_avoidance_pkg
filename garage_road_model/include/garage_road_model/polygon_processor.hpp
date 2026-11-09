#pragma once

#include "garage_road_model/types.hpp"

#include <vector>

namespace garage_road_model
{

// 检查边界点是否构成可用多边形。
std::vector<Eigen::Vector2d> validatePolygon(const std::vector<Eigen::Vector2d> & boundary);

// 判断点是否在多边形内，允许贴着边界。
bool pointInsidePolygon(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polygon,
  double tolerance);

// 点到折线的最短距离。
double distanceToEdges(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polyline,
  bool closed);

// 判断两条线段是否相交。
bool segmentsIntersect(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c,
  const Eigen::Vector2d & d);

// 判断折线是否自交。
bool polylineSelfIntersects(
  const std::vector<Eigen::Vector2d> & points,
  bool closed);

// 按固定间距重采样折线。
std::vector<Eigen::Vector2d> resamplePolyline(
  const std::vector<Eigen::Vector2d> & points,
  double spacing);

}  // namespace garage_road_model
