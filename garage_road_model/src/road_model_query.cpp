#include "garage_road_model/road_model_query.hpp"

#include "garage_road_model/polygon_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace garage_road_model
{
namespace
{

struct SegmentHit
{
  double distance{std::numeric_limits<double>::infinity()};
  double s{0.0};
  double gap_s{0.0};
  Eigen::Vector2d point{Eigen::Vector2d::Zero()};
  double yaw{0.0};
};

Eigen::Vector2d positionOf(const RoadSampleData & sample)
{
  return Eigen::Vector2d(sample.x, sample.y);
}

double wrapAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

SegmentHit projectOnto(const Eigen::Vector2d & point, const std::vector<RoadSampleData> & samples)
{
  SegmentHit best;
  SegmentHit second;
  if (samples.size() < 2) {
    return best;
  }
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    const Eigen::Vector2d start = positionOf(samples[i]);
    const Eigen::Vector2d end = positionOf(samples[i + 1]);
    const Eigen::Vector2d edge = end - start;
    const double length_sq = edge.squaredNorm();
    double ratio = 0.0;
    if (length_sq > 1e-16) {
      ratio = std::clamp((point - start).dot(edge) / length_sq, 0.0, 1.0);
    }
    const Eigen::Vector2d foot = start + ratio * edge;
    const double distance = (point - foot).norm();
    const double s = samples[i].s + ratio * (samples[i + 1].s - samples[i].s);
    const double yaw = std::atan2(edge.y(), edge.x());
    if (distance < best.distance) {
      second = best;
      best.distance = distance;
      best.s = s;
      best.point = foot;
      best.yaw = yaw;
    } else if (distance < second.distance) {
      second.distance = distance;
      second.s = s;
    }
  }
  best.gap_s = second.distance < 1e8 ? std::abs(best.s - second.s) : 0.0;
  if (best.gap_s > 0.0 && second.distance > best.distance + 0.2) {
    best.gap_s = 0.0;
  }
  return best;
}

}  // namespace

RoadModelQuery::RoadModelQuery(RoadModelData model)
: model_(std::move(model))
{
}

const SideBoundary & RoadModelQuery::boundaryOf(SideId side) const
{
  return side == SideId::A ? model_.side_a : model_.side_b;
}

SideProjection RoadModelQuery::nearestSide(const Eigen::Vector2d & point) const
{
  SideProjection result;
  if (!model_.valid) {
    result.reason = "道路无效";
    return result;
  }
  const SegmentHit side_a = projectOnto(point, model_.side_a.samples);
  const SegmentHit side_b = projectOnto(point, model_.side_b.samples);
  if (!std::isfinite(side_a.distance) || !std::isfinite(side_b.distance)) {
    result.reason = "侧边无法投影";
    return result;
  }
  const bool choose_a = side_a.distance <= side_b.distance;
  const SegmentHit & chosen = choose_a ? side_a : side_b;
  const SegmentHit & other = choose_a ? side_b : side_a;
  if (std::abs(side_a.distance - side_b.distance) <= 0.05 || chosen.gap_s > 1.0) {
    result.status = QueryStatus::Ambiguous;
    result.reason = "近侧边不唯一";
    return result;
  }
  result.status = QueryStatus::Ok;
  result.side = choose_a ? SideId::A : SideId::B;
  result.s = chosen.s;
  result.point = chosen.point;
  result.distance = chosen.distance;
  result.yaw = chosen.yaw;
  (void)other;
  return result;
}

SidePose RoadModelQuery::evaluateSide(SideId side, double s) const
{
  SidePose pose;
  const auto & boundary = boundaryOf(side);
  if (!model_.valid || boundary.samples.size() < 2) {
    pose.reason = "道路无效";
    return pose;
  }
  const double begin = boundary.samples.front().s;
  const double end = boundary.samples.back().s;
  if (s < begin - 1e-6 || s > end + 1e-6) {
    pose.status = QueryStatus::OutOfRange;
    pose.reason = "弧长超出侧边";
    return pose;
  }
  int upper = 1;
  while (upper + 1 < static_cast<int>(boundary.samples.size()) &&
    boundary.samples[static_cast<size_t>(upper)].s < s)
  {
    ++upper;
  }
  const auto & left = boundary.samples[static_cast<size_t>(upper - 1)];
  const auto & right = boundary.samples[static_cast<size_t>(upper)];
  const double span = right.s - left.s;
  const double ratio = span < 1e-9 ? 0.0 : std::clamp((s - left.s) / span, 0.0, 1.0);
  pose.status = QueryStatus::Ok;
  pose.position = (1.0 - ratio) * positionOf(left) + ratio * positionOf(right);
  const double yaw = left.yaw + ratio * wrapAngle(right.yaw - left.yaw);
  pose.yaw = yaw;
  pose.tangent = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
  pose.s = s;
  pose.corner = left.corner || right.corner || std::abs(wrapAngle(right.yaw - left.yaw)) > 0.5;
  return pose;
}

InwardNormal RoadModelQuery::inwardNormal(SideId side, double s) const
{
  InwardNormal result;
  const SidePose pose = evaluateSide(side, s);
  if (pose.status != QueryStatus::Ok) {
    result.status = pose.status;
    result.reason = pose.reason;
    return result;
  }
  if (pose.corner) {
    result.status = QueryStatus::Ambiguous;
    result.reason = "拐角处没有唯一法线";
    return result;
  }
  const Eigen::Vector2d left(-pose.tangent.y(), pose.tangent.x());
  result.direction = side == SideId::A ? left : Eigen::Vector2d(-left.x(), -left.y());
  result.status = QueryStatus::Ok;
  return result;
}

CrossSection RoadModelQuery::localCrossSection(SideId side, double s) const
{
  CrossSection section;
  const InwardNormal normal = inwardNormal(side, s);
  const SidePose pose = evaluateSide(side, s);
  if (normal.status != QueryStatus::Ok || pose.status != QueryStatus::Ok) {
    section.status = normal.status == QueryStatus::Ok ? pose.status : normal.status;
    section.reason = normal.status == QueryStatus::Ok ? pose.reason : normal.reason;
    return section;
  }
  double best = std::numeric_limits<double>::infinity();
  const auto & polygon = model_.boundary;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d a = polygon[i];
    const Eigen::Vector2d b = polygon[(i + 1) % polygon.size()];
    const Eigen::Vector2d edge = b - a;
    const double det = normal.direction.x() * edge.y() - normal.direction.y() * edge.x();
    if (std::abs(det) < 1e-12) {
      continue;
    }
    const Eigen::Vector2d delta = a - pose.position;
    const double distance = (delta.x() * edge.y() - delta.y() * edge.x()) / det;
    const double ratio = (delta.x() * normal.direction.y() - delta.y() * normal.direction.x()) / det;
    if (distance > 0.05 && ratio >= -1e-6 && ratio <= 1.0 + 1e-6) {
      best = std::min(best, distance);
    }
  }
  if (!std::isfinite(best)) {
    section.reason = "没有朝内的交点";
    return section;
  }
  section.status = QueryStatus::Ok;
  section.s = s;
  section.width = best;
  section.near_point = pose.position;
  section.far_point = pose.position + best * normal.direction;
  return section;
}

}  // namespace garage_road_model
