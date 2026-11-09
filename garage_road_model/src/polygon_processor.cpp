#include "garage_road_model/polygon_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace garage_road_model
{
namespace
{

constexpr double kEps = 1e-9;

// 两点之间的距离。
double distance(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return (a - b).norm();
}

// 多边形的有向面积。
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

}  // namespace

// 检查边界点是否构成可用多边形。
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

// 把闭合边界切成两个端口和两条长边。
namespace
{

struct Chain
{
  int start{0};
  int end{0};
  double length{0.0};
  std::vector<Eigen::Vector2d> points;
};

// 把下标折回到闭合环上。
int wrap(int index, int count)
{
  return (index % count + count) % count;
}


// 计算折点处的转角。
double turnAngle(
  const Eigen::Vector2d & before,
  const Eigen::Vector2d & at,
  const Eigen::Vector2d & after)
{
  const Eigen::Vector2d incoming = at - before;
  const Eigen::Vector2d outgoing = after - at;
  if (incoming.squaredNorm() < 1e-16 || outgoing.squaredNorm() < 1e-16) {
    return 0.0;
  }
  return std::abs(std::atan2(incoming.x() * outgoing.y() - incoming.y() * outgoing.x(), incoming.dot(outgoing)));
}

// 按下标从环上切出一段点。
std::vector<Eigen::Vector2d> slice(
  const std::vector<Eigen::Vector2d> & points,
  int begin,
  int end)
{
  const int count = static_cast<int>(points.size());
  std::vector<Eigen::Vector2d> chain;
  int index = begin;
  for (int guard = 0; guard <= count; ++guard) {
    chain.push_back(points[static_cast<size_t>(index)]);
    if (index == end) {
      return chain;
    }
    index = wrap(index + 1, count);
  }
  throw ModelError("端口识别", "侧边索引没有闭合");
}

// 折线的总长度。
double polylineLength(const std::vector<Eigen::Vector2d> & points)
{
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

// 把一段点接到链上，并去掉重复的端点。
void appendChain(std::vector<Eigen::Vector2d> & output, const std::vector<Eigen::Vector2d> & part)
{
  if (part.empty()) {
    return;
  }
  size_t begin = 0;
  if (!output.empty() && (output.back() - part.front()).norm() < 1e-8) {
    begin = 1;
  }
  output.insert(output.end(), part.begin() + static_cast<std::ptrdiff_t>(begin), part.end());
}

// 去掉共线点，简化闭合边界。
std::vector<int> simplifyRing(const std::vector<Eigen::Vector2d> & points, double tolerance)
{
  const int count = static_cast<int>(points.size());
  std::vector<int> keep{0};
  int start = 0;
  for (int guard = 0; guard < count; ++guard) {
    int best = wrap(start + 1, count);
    for (int step = 2; step < count; ++step) {
      const int candidate = wrap(start + step, count);
      if (candidate == keep.front()) {
        break;
      }
      bool collinear = true;
      for (int mid_step = 1; mid_step < step; ++mid_step) {
        const int mid = wrap(start + mid_step, count);
        const double deviation = distanceToEdges(
          points[static_cast<size_t>(mid)],
          {points[static_cast<size_t>(start)], points[static_cast<size_t>(candidate)]},
          false);
        if (deviation > tolerance) {
          collinear = false;
          break;
        }
      }
      if (!collinear) {
        break;
      }
      best = candidate;
    }
    if (best == keep.front()) {
      break;
    }
    keep.push_back(best);
    start = best;
    if (static_cast<int>(keep.size()) >= count) {
      break;
    }
  }
  return keep;
}

// 取方向向量的左侧单位法线。
Eigen::Vector2d leftNormal(const Eigen::Vector2d & direction)
{
  const double length = direction.norm();
  if (length < 1e-9) {
    return Eigen::Vector2d::Zero();
  }
  return Eigen::Vector2d(-direction.y() / length, direction.x() / length);
}

// 判断射线是否打到线段。
bool rayHitsSegment(
  const Eigen::Vector2d & origin,
  const Eigen::Vector2d & direction,
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  double & distance)
{
  const Eigen::Vector2d edge = b - a;
  const double det = direction.x() * edge.y() - direction.y() * edge.x();
  if (std::abs(det) < 1e-12) {
    return false;
  }
  const Eigen::Vector2d delta = a - origin;
  distance = (delta.x() * edge.y() - delta.y() * edge.x()) / det;
  const double ratio = (delta.x() * direction.y() - delta.y() * direction.x()) / det;
  return distance > 0.05 && ratio >= -1e-6 && ratio <= 1.0 + 1e-6;
}

// 判断线段是否留在多边形内部。
bool segmentStaysInside(
  const Eigen::Vector2d & start,
  const Eigen::Vector2d & stop,
  const std::vector<Eigen::Vector2d> & polygon)
{
  for (double ratio : {0.25, 0.5, 0.75}) {
    if (!pointInsidePolygon(start + ratio * (stop - start), polygon, 0.05)) {
      return false;
    }
  }
  return true;
}

// 判断两条边之间的宽度是否像一条通道。
bool widthLooksLikeCorridor(
  const std::vector<Eigen::Vector2d> & from_side,
  const std::vector<Eigen::Vector2d> & to_side,
  const std::vector<Eigen::Vector2d> & polygon,
  const std::vector<Eigen::Vector2d> & corners,
  double cap_length,
  double success_fraction,
  double & score_fraction)
{
  int checked = 0;
  int hits = 0;
  std::vector<double> widths;
  double traveled = 0.0;
  double next_sample = 0.0;
  for (size_t i = 1; i < from_side.size(); ++i) {
    const Eigen::Vector2d edge = from_side[i] - from_side[i - 1];
    const double length = edge.norm();
    if (length < 1e-9) {
      continue;
    }
    const Eigen::Vector2d normal = leftNormal(edge);
    while (next_sample <= traveled + length + 1e-9) {
      const double ratio = std::clamp((next_sample - traveled) / length, 0.0, 1.0);
      const Eigen::Vector2d point = from_side[i - 1] + ratio * edge;
      next_sample += 0.5;
      bool near_corner = false;
      for (const auto & corner : corners) {
        if ((point - corner).norm() < 0.35) {
          near_corner = true;
          break;
        }
      }
      if (near_corner || normal.squaredNorm() < 0.5) {
        continue;
      }
      ++checked;
      double best = std::numeric_limits<double>::infinity();
      for (size_t j = 1; j < to_side.size(); ++j) {
        double distance = 0.0;
        if (rayHitsSegment(point, normal, to_side[j - 1], to_side[j], distance)) {
          best = std::min(best, distance);
        }
      }
      if (!std::isfinite(best)) {
        continue;
      }
      const Eigen::Vector2d hit = point + best * normal;
      if (!segmentStaysInside(point, hit, polygon)) {
        continue;
      }
      ++hits;
      widths.push_back(best);
    }
    traveled += length;
  }
  score_fraction = checked == 0 ? 0.0 : static_cast<double>(hits) / static_cast<double>(checked);
  if (checked < 2 || score_fraction < success_fraction || widths.empty()) {
    return false;
  }
  std::sort(widths.begin(), widths.end());
  const double median = widths[widths.size() / 2];
  return median >= 0.3 * cap_length && median <= 3.0 * cap_length;
}

// 取两个端口之间的那条侧边。
std::vector<Eigen::Vector2d> betweenCaps(
  const std::vector<Chain> & chains,
  int from_cap,
  int to_cap)
{
  std::vector<Eigen::Vector2d> side;
  const int count = static_cast<int>(chains.size());
  int index = wrap(from_cap + 1, count);
  for (int guard = 0; index != to_cap && guard < count; ++guard) {
    appendChain(side, chains[static_cast<size_t>(index)].points);
    index = wrap(index + 1, count);
  }
  return side;
}

// 取点链的中点。
Eigen::Vector2d chainMidpoint(const std::vector<Eigen::Vector2d> & points)
{
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (const auto & point : points) {
    sum += point;
  }
  return points.empty() ? sum : sum / static_cast<double>(points.size());
}

// 按坐标比较两个点的先后。
bool smallerPoint(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  if (std::abs(a.x() - b.x()) > 1e-9) {
    return a.x() < b.x();
  }
  return a.y() < b.y();
}

}  // namespace

// 把闭合边界切成两个端口和两条长边。失败时抛出错误，不硬选。
BoundarySplit splitBoundary(
  const std::vector<Eigen::Vector2d> & boundary,
  const ModelConfig & config)
{
  if (config.collinear_tolerance <= 0.0 || config.cap_side_ratio <= 0.0 ||
    config.cap_side_ratio >= 1.0)
  {
    throw ModelError("端口识别", "端头参数不合法");
  }
  std::vector<Eigen::Vector2d> ring = boundary;
  if (polygonArea(ring) < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  const int count = static_cast<int>(ring.size());
  if (count < 4) {
    throw ModelError("端口识别", "端头不明确");
  }

  const std::vector<int> keep = simplifyRing(ring, config.collinear_tolerance);
  if (keep.size() < 4) {
    throw ModelError("端口识别", "端头不明确");
  }
  std::vector<int> corner_slots;
  std::vector<Eigen::Vector2d> corner_points;
  const int simplified = static_cast<int>(keep.size());
  for (int i = 0; i < simplified; ++i) {
    const double turn = turnAngle(
      ring[static_cast<size_t>(keep[static_cast<size_t>(wrap(i - 1, simplified))])],
      ring[static_cast<size_t>(keep[static_cast<size_t>(i)])],
      ring[static_cast<size_t>(keep[static_cast<size_t>(wrap(i + 1, simplified))])]);
    if (turn >= config.corner_angle_rad) {
      corner_slots.push_back(i);
      corner_points.push_back(ring[static_cast<size_t>(keep[static_cast<size_t>(i)])]);
    }
  }
  if (corner_slots.size() < 4) {
    throw ModelError("端口识别", "端头不明确");
  }

  std::vector<Chain> chains;
  const int corner_count = static_cast<int>(corner_slots.size());
  for (int i = 0; i < corner_count; ++i) {
    Chain chain;
    chain.start = keep[static_cast<size_t>(corner_slots[static_cast<size_t>(i)])];
    chain.end = keep[static_cast<size_t>(corner_slots[static_cast<size_t>((i + 1) % corner_count)])];
    chain.points = slice(ring, chain.start, chain.end);
    chain.length = polylineLength(chain.points);
    if (chain.points.size() < 2 || chain.length < 1e-4) {
      throw ModelError("端口识别", "端头不明确");
    }
    chains.push_back(std::move(chain));
  }

  struct Candidate
  {
    int first{0};
    int second{0};
    double score{0.0};
  };
  std::vector<Candidate> candidates;
  const int chain_count = static_cast<int>(chains.size());
  for (int i = 0; i < chain_count; ++i) {
    for (int j = i + 1; j < chain_count; ++j) {
      const int between = j - i - 1;
      const int around = chain_count - between - 2;
      if (between < 1 || around < 1) {
        continue;
      }
      const auto side_ij = betweenCaps(chains, i, j);
      const auto side_ji = betweenCaps(chains, j, i);
      const double side_ij_length = polylineLength(side_ij);
      const double side_ji_length = polylineLength(side_ji);
      const double cap_limit = config.cap_side_ratio * std::min(side_ij_length, side_ji_length);
      if (side_ij.size() < 2 || side_ji.size() < 2 ||
        chains[static_cast<size_t>(i)].length > cap_limit ||
        chains[static_cast<size_t>(j)].length > cap_limit)
      {
        continue;
      }
      const double mean_cap = 0.5 * (
        chains[static_cast<size_t>(i)].length + chains[static_cast<size_t>(j)].length);
      double fraction_ij = 0.0;
      double fraction_ji = 0.0;
      if (!widthLooksLikeCorridor(
          side_ij, side_ji, ring, corner_points, mean_cap, config.width_success_fraction, fraction_ij) ||
        !widthLooksLikeCorridor(
          side_ji, side_ij, ring, corner_points, mean_cap, config.width_success_fraction, fraction_ji))
      {
        continue;
      }
      bool junctions_ok = true;
      for (const int slot : {i, j, (i + between + 1) % chain_count, (j + around + 1) % chain_count}) {
        const auto & chain = chains[static_cast<size_t>(slot)];
        const int previous = wrap(chain.start - 1, count);
        const int next = wrap(chain.start + 1, count);
        if (turnAngle(ring[static_cast<size_t>(previous)], ring[static_cast<size_t>(chain.start)],
          ring[static_cast<size_t>(next)]) < config.junction_angle_rad)
        {
          junctions_ok = false;
          break;
        }
      }
      if (!junctions_ok) {
        continue;
      }
      Candidate candidate;
      candidate.first = i;
      candidate.second = j;
      candidate.score = std::min(side_ij_length, side_ji_length) /
        std::max(chains[static_cast<size_t>(i)].length, chains[static_cast<size_t>(j)].length);
      candidate.score *= 0.5 * (fraction_ij + fraction_ji);
      candidates.push_back(candidate);
    }
  }

  if (candidates.empty()) {
    throw ModelError("端口识别", "端口歧义，没有可靠端头");
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return a.score > b.score;
    });
  if (candidates.size() > 1 && candidates[0].score < candidates[1].score * 1.2) {
    throw ModelError("端口识别", "端口歧义，候选得分接近");
  }

  const int first = candidates.front().first;
  const int second = candidates.front().second;
  const Eigen::Vector2d first_mid = chainMidpoint(chains[static_cast<size_t>(first)].points);
  const Eigen::Vector2d second_mid = chainMidpoint(chains[static_cast<size_t>(second)].points);
  const int start = smallerPoint(first_mid, second_mid) ? first : second;
  const int end = start == first ? second : first;

  BoundarySplit split;
  split.boundary = ring;
  split.start_cap = chains[static_cast<size_t>(start)].points;
  split.end_cap = chains[static_cast<size_t>(end)].points;
  split.side_a = betweenCaps(chains, start, end);
  split.side_b = betweenCaps(chains, end, start);
  std::reverse(split.side_b.begin(), split.side_b.end());
  if (split.side_a.size() < 2 || split.side_b.size() < 2) {
    throw ModelError("端口识别", "端口歧义，侧边不完整");
  }
  return split;
}

}  // namespace garage_road_model
