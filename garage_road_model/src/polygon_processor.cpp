#include "garage_road_model/polygon_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace garage_road_model
{
namespace
{

constexpr double kEps = 1e-9;

double distance(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return (a - b).norm();
}

double orient(const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & c)
{
  return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

double polygonArea(const std::vector<Eigen::Vector2d> & points)
{
  double area = 0.0;
  const int count = static_cast<int>(points.size());
  for (int i = 0; i < count; ++i) {
    const auto & p = points[i];
    const auto & q = points[(i + 1) % count];
    area += p.x() * q.y() - q.x() * p.y();
  }
  return 0.5 * area;
}

bool containsRay(const Eigen::Vector2d & point, const std::vector<Eigen::Vector2d> & polygon)
{
  bool inside = false;
  const int count = static_cast<int>(polygon.size());
  int j = count - 1;
  for (int i = 0; i < count; ++i) {
    const double yi = polygon[i].y();
    const double yj = polygon[j].y();
    const bool crosses = (yi > point.y()) != (yj > point.y());
    if (crosses) {
      const double denom = std::abs(yj - yi) < 1e-15 ? 1e-15 : (yj - yi);
      const double x_hit =
        (polygon[j].x() - polygon[i].x()) * (point.y() - yi) / denom + polygon[i].x();
      if (point.x() < x_hit) {
        inside = !inside;
      }
    }
    j = i;
  }
  return inside;
}

double distanceToSegment(
  const Eigen::Vector2d & point,
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & end)
{
  const Eigen::Vector2d edge = end - start;
  const double length_sq = edge.squaredNorm();
  if (length_sq < 1e-18) {
    return distance(point, start);
  }
  double ratio = (point - start).dot(edge) / length_sq;
  ratio = std::clamp(ratio, 0.0, 1.0);
  return distance(point, start + ratio * edge);
}

}  // namespace

std::vector<Eigen::Vector2d> validatePolygon(const std::vector<Eigen::Vector2d> & boundary)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(boundary.size());
  for (const auto & point : boundary) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      throw ModelError("多边形校验", "坐标不是有限值");
    }
    points.push_back(point);
  }
  if (points.size() >= 2 && distance(points.front(), points.back()) <= kEps) {
    points.pop_back();
  }

  std::vector<Eigen::Vector2d> cleaned;
  for (const auto & point : points) {
    if (cleaned.empty() || distance(cleaned.back(), point) > kEps) {
      cleaned.push_back(point);
    }
  }
  if (cleaned.size() >= 2 && distance(cleaned.front(), cleaned.back()) <= kEps) {
    cleaned.pop_back();
  }

  int unique = 0;
  for (size_t i = 0; i < cleaned.size(); ++i) {
    bool seen = false;
    for (size_t j = 0; j < i; ++j) {
      if (distance(cleaned[i], cleaned[j]) <= kEps) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      ++unique;
    }
  }
  if (unique < 3) {
    throw ModelError("多边形校验", "至少需要三个不同顶点");
  }
  if (polylineSelfIntersects(cleaned, true)) {
    throw ModelError("多边形校验", "多边形自交");
  }
  if (std::abs(polygonArea(cleaned)) < 1e-8) {
    throw ModelError("多边形校验", "零面积");
  }
  return cleaned;
}

bool pointInsidePolygon(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polygon,
  double tolerance)
{
  if (containsRay(point, polygon)) {
    return true;
  }
  return distanceToEdges(point, polygon, true) <= tolerance;
}

double distanceToEdges(
  const Eigen::Vector2d & point,
  const std::vector<Eigen::Vector2d> & polyline,
  bool closed)
{
  if (polyline.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  const int limit = closed ? static_cast<int>(polyline.size()) : static_cast<int>(polyline.size()) - 1;
  for (int i = 0; i < limit; ++i) {
    best = std::min(
      best, distanceToSegment(point, polyline[i], polyline[(i + 1) % polyline.size()]));
  }
  return best;
}

bool segmentsIntersect(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c,
  const Eigen::Vector2d & d)
{
  const double o1 = orient(a, b, c);
  const double o2 = orient(a, b, d);
  const double o3 = orient(c, d, a);
  const double o4 = orient(c, d, b);
  return o1 * o2 < 0.0 && o3 * o4 < 0.0;
}

bool polylineSelfIntersects(const std::vector<Eigen::Vector2d> & points, bool closed)
{
  const int count = static_cast<int>(points.size());
  if (count < 4) {
    return false;
  }
  const int edge_count = closed ? count : count - 1;
  for (int i = 0; i < edge_count; ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % count];
    for (int j = i + 1; j < edge_count; ++j) {
      if (j == i + 1) {
        continue;
      }
      if (closed && i == 0 && j == edge_count - 1) {
        continue;
      }
      const auto & c = points[j];
      const auto & d = points[(j + 1) % count];
      if (segmentsIntersect(a, b, c, d)) {
        return true;
      }
    }
  }
  return false;
}

std::vector<Eigen::Vector2d> resamplePolyline(
  const std::vector<Eigen::Vector2d> & points,
  double spacing)
{
  if (points.size() < 2 || spacing <= 0.0) {
    return points;
  }
  std::vector<Eigen::Vector2d> output;
  output.push_back(points.front());
  double next_distance = spacing;
  double traveled = 0.0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const Eigen::Vector2d start = points[i];
    const Eigen::Vector2d end = points[i + 1];
    const double segment = distance(start, end);
    if (segment < 1e-12) {
      continue;
    }
    while (traveled + segment >= next_distance - 1e-9) {
      const double ratio = std::clamp((next_distance - traveled) / segment, 0.0, 1.0);
      output.push_back(start + ratio * (end - start));
      next_distance += spacing;
    }
    traveled += segment;
  }
  if (distance(output.back(), points.back()) > 1e-3) {
    output.push_back(points.back());
  }
  return output;
}

}  // namespace garage_road_model
