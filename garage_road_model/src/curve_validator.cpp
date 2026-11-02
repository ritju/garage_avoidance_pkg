#include "garage_road_model/curve_validator.hpp"

#include "garage_road_model/polygon_processor.hpp"

#include <vector>

namespace garage_road_model
{

bool rejectFittedSides(
  const SideBoundary & side_a,
  const SideBoundary & side_b,
  std::string & reason)
{
  auto pointsOf = [](const SideBoundary & side) {
      std::vector<Eigen::Vector2d> points;
      points.reserve(side.samples.size());
      for (const auto & sample : side.samples) {
        points.emplace_back(sample.x, sample.y);
      }
      return points;
    };
  const auto first = pointsOf(side_a);
  const auto second = pointsOf(side_b);
  if (first.size() < 2 || second.size() < 2) {
    reason = "拟合侧边点数不足";
    return true;
  }
  if (polylineSelfIntersects(first, false) || polylineSelfIntersects(second, false)) {
    reason = "拟合侧边自交";
    return true;
  }
  const int first_count = static_cast<int>(first.size());
  const int second_count = static_cast<int>(second.size());
  for (int i = 0; i + 1 < first_count; ++i) {
    for (int j = 0; j + 1 < second_count; ++j) {
      if (segmentsIntersect(first[static_cast<size_t>(i)], first[static_cast<size_t>(i + 1)],
        second[static_cast<size_t>(j)], second[static_cast<size_t>(j + 1)]))
      {
        reason = "两侧拟合线交叉";
        return true;
      }
    }
  }
  return false;
}

}  // namespace garage_road_model
