#pragma once

#include "garage_road_model/types.hpp"

#include <vector>

namespace garage_road_model
{

std::vector<Eigen::Vector2d> validatePolygon(const std::vector<Eigen::Vector2d> & boundary);

bool pointInsidePolygon(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polygon,
  double tolerance);

double distanceToEdges(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polyline,
  bool closed);

bool segmentsIntersect(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c,
  const Eigen::Vector2d & d);

bool polylineSelfIntersects(
  const std::vector<Eigen::Vector2d> & points,
  bool closed);

std::vector<Eigen::Vector2d> resamplePolyline(
  const std::vector<Eigen::Vector2d> & points,
  double spacing);

}  // namespace garage_road_model
