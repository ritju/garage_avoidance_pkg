#include "garage_utils/footprint_check.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace garage_utils_pkg
{
namespace
{
using XY = std::pair<double, double>;
double cross(const XY & a, const XY & b, const XY & c)
{
  return (b.first-a.first)*(c.second-a.second)-(b.second-a.second)*(c.first-a.first);
}
std::vector<XY> hull(std::vector<XY> points)
{
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) {return points;}
  std::vector<XY> result(2 * points.size());
  size_t n = 0;
  for (const auto & p : points) {
    while (n >= 2 && cross(result[n-2], result[n-1], p) <= 0) {--n;}
    result[n++] = p;
  }
  const size_t lower = n + 1;
  for (size_t i = points.size()-1; i > 0; --i) {
    const auto & p = points[i-1];
    while (n >= lower && cross(result[n-2], result[n-1], p) <= 0) {--n;}
    result[n++] = p;
  }
  result.resize(n-1);
  return result;
}

bool areaFree(const nav2_msgs::msg::Costmap & map, const std::vector<XY> & polygon,
  double padding, std::string & reason)
{
  double min_x = polygon[0].first, max_x = min_x;
  double min_y = polygon[0].second, max_y = min_y;
  for (const auto & p : polygon) {
    min_x = std::min(min_x, p.first); max_x = std::max(max_x, p.first);
    min_y = std::min(min_y, p.second); max_y = std::max(max_y, p.second);
  }
  min_x -= padding; min_y -= padding; max_x += padding; max_y += padding;
  const double res = map.metadata.resolution;
  if (min_x < 0 || min_y < 0 || max_x >= map.metadata.size_x*res ||
    max_y >= map.metadata.size_y*res)
  {
    reason = "车身扫过区域超出代价地图";
    return false;
  }
  struct Axis {double x, y, low, high;};
  std::vector<Axis> axes;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i+1)%polygon.size()];
    const double length = std::hypot(b.first-a.first, b.second-a.second);
    Axis axis{-(b.second-a.second)/length, (b.first-a.first)/length, 0, 0};
    axis.low = axis.high = axis.x*polygon[0].first + axis.y*polygon[0].second;
    for (const auto & p : polygon) {
      const double value = axis.x*p.first + axis.y*p.second;
      axis.low = std::min(axis.low, value); axis.high = std::max(axis.high, value);
    }
    const double margin = padding + res*0.5*(std::abs(axis.x)+std::abs(axis.y));
    axis.low -= margin; axis.high += margin;
    axes.push_back(axis);
  }
  // 包括边界刚好接触的格子。只有障碍格才做多边形相交计算。
  const int x0 = std::max(0, static_cast<int>(std::floor(min_x/res-1e-9)));
  const int y0 = std::max(0, static_cast<int>(std::floor(min_y/res-1e-9)));
  const int x1 = static_cast<int>(std::floor(max_x/res));
  const int y1 = static_cast<int>(std::floor(max_y/res));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const auto cost = map.data[static_cast<size_t>(y)*map.metadata.size_x+x];
      if (cost < 254) {continue;}
      bool overlaps = true;
      for (const auto & axis : axes) {
        const double value = axis.x*(x+0.5)*res + axis.y*(y+0.5)*res;
        if (value < axis.low-1e-9 || value > axis.high+1e-9) {
          overlaps = false;
          break;
        }
      }
      if (overlaps) {
        reason = cost == 255 ? "车身扫过未知区域" : "车身扫过障碍";
        return false;
      }
    }
  }
  return true;
}
}

FootprintCheck::FootprintCheck(const nav2_msgs::msg::Costmap & map,
  const geometry_msgs::msg::Polygon & footprint) : map_(map), footprint_(footprint)
{
  const auto & m = map.metadata;
  if (map.header.frame_id != "map" || !std::isfinite(m.resolution) || m.resolution <= 0 ||
    m.size_x == 0 || m.size_y == 0 || map.data.size() != static_cast<size_t>(m.size_x)*m.size_y)
  {throw std::runtime_error("代价地图无效或坐标系不是 map");}
  std::vector<XY> vertices;
  for (const auto & p : footprint.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      throw std::runtime_error("footprint 坐标无效");
    }
    radius_ = std::max(radius_, std::hypot(double(p.x), double(p.y)));
    vertices.emplace_back(p.x, p.y);
  }
  if (hull(vertices).size() < 3) {throw std::runtime_error("footprint 无效");}
}

bool FootprintCheck::motionFree(const geometry_msgs::msg::Pose & from,
  const geometry_msgs::msg::Pose & to, std::string & reason) const
{
  const double yaw0 = tf2::getYaw(from.orientation);
  const double angle = std::remainder(tf2::getYaw(to.orientation)-yaw0, 2*M_PI);
  // 同向平移只检查一次凸包。转动时按车角移动不超过一格来分段。
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle)*radius_/map_.metadata.resolution)));
  const double step_angle = angle/steps;
  const double padding = radius_*step_angle*step_angle/8.0;
  const double map_yaw = tf2::getYaw(map_.metadata.origin.orientation);
  const double mc = std::cos(map_yaw), ms = std::sin(map_yaw);
  for (int i = 0; i < steps; ++i) {
    std::vector<XY> vertices;
    for (int end = i; end <= i+1; ++end) {
      const double t = static_cast<double>(end)/steps;
      const double x = from.position.x + t*(to.position.x-from.position.x);
      const double y = from.position.y + t*(to.position.y-from.position.y);
      const double yaw = yaw0 + t*angle;
      for (const auto & p : footprint_.points) {
        const double wx = x + std::cos(yaw)*p.x-std::sin(yaw)*p.y-map_.metadata.origin.position.x;
        const double wy = y + std::sin(yaw)*p.x+std::cos(yaw)*p.y-map_.metadata.origin.position.y;
        vertices.emplace_back(mc*wx+ms*wy, -ms*wx+mc*wy);
      }
    }
    // 弧线与弦之间的最大偏差也计入，覆盖两次姿态之间的转动。
    if (!areaFree(map_, hull(std::move(vertices)), padding, reason)) {return false;}
  }
  return true;
}
}
