#include "garage_road_model/geometry.hpp"
#include "garage_road_model/polygon_processor.hpp"
#include "garage_road_model/road_model_builder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace garage_road_model
{
namespace
{

// 顶点处的转角。共线时为 0。
double pieceTurn(const Eigen::Vector2d & before, const Eigen::Vector2d & at, const Eigen::Vector2d & after)
{
  const Eigen::Vector2d incoming = at - before;
  const Eigen::Vector2d outgoing = after - at;
  if (incoming.squaredNorm() < 1e-16 || outgoing.squaredNorm() < 1e-16) {
    return 0.0;
  }
  return std::abs(std::atan2(
      incoming.x() * outgoing.y() - incoming.y() * outgoing.x(), incoming.dot(outgoing)));
}

// 沿原始长边按间距采样。拐角顶点一定保留，并标成没有唯一法线。
SideBoundary sampleSide(const std::vector<Eigen::Vector2d> & chain, const ModelConfig & config)
{
  const double spacing = config.output_sample_spacing;
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw ModelError("输出采样", "output_sample_spacing 必须是有限正数");
  }
  if (chain.size() < 2) {
    throw ModelError("曲线采样", "侧边点数不足");
  }

  std::vector<double> vertex_s(chain.size(), 0.0);
  std::vector<char> vertex_corner(chain.size(), 0);
  for (size_t i = 1; i < chain.size(); ++i) {
    vertex_s[i] = vertex_s[i - 1] + (chain[i] - chain[i - 1]).norm();
  }
  for (size_t i = 1; i + 1 < chain.size(); ++i) {
    if (pieceTurn(chain[i - 1], chain[i], chain[i + 1]) >= config.piece_angle_rad) {
      vertex_corner[i] = 1;
    }
  }

  const double length = vertex_s.back();
  if (length < 1e-4) {
    throw ModelError("曲线采样", "侧边长度过短");
  }
  if (length / spacing > 1000000.0) {
    throw ModelError("输出采样", "输出点数量超过上限");
  }

  std::vector<double> targets{0.0, length};
  for (size_t k = 1; static_cast<double>(k) * spacing < length; ++k) {
    targets.push_back(static_cast<double>(k) * spacing);
  }
  for (size_t i = 0; i < chain.size(); ++i) {
    if (vertex_corner[i]) {
      targets.push_back(vertex_s[i]);
    }
  }
  std::sort(targets.begin(), targets.end());

  SideBoundary side;
  side.raw = chain;
  for (double s : targets) {
    if (!side.samples.empty() && s - side.samples.back().s <= 1e-6) {
      continue;
    }
    size_t upper = 1;
    while (upper + 1 < chain.size() && vertex_s[upper] < s - 1e-9) {
      ++upper;
    }
    const double span = vertex_s[upper] - vertex_s[upper - 1];
    const double ratio = span < 1e-12 ? 0.0 : std::clamp((s - vertex_s[upper - 1]) / span, 0.0, 1.0);
    const Eigen::Vector2d point =
      chain[upper - 1] + ratio * (chain[upper] - chain[upper - 1]);
    const Eigen::Vector2d direction = chain[upper] - chain[upper - 1];
    RoadSampleData sample;
    sample.x = point.x();
    sample.y = point.y();
    sample.yaw = std::atan2(direction.y(), direction.x());
    sample.s = s;
    for (size_t i = 0; i < chain.size(); ++i) {
      if (vertex_corner[i] && std::abs(vertex_s[i] - s) <= 1e-6) {
        sample.corner = true;
        break;
      }
    }
    side.samples.push_back(sample);
  }
  if (side.samples.size() < 2) {
    throw ModelError("曲线采样", "侧边采样失败");
  }
  side.length = side.samples.back().s;
  return side;
}

}  // namespace

// 校验边界、切出长边，再沿两条长边采样。
RoadModelData RoadModelBuilder::build(
  const std::vector<Eigen::Vector2d> & boundary,
  uint32_t source_index,
  const ModelConfig & config) const
{
  RoadModelData model;
  model.source_index = source_index;
  const auto started = std::chrono::steady_clock::now();
  auto stamp = started;
  auto mark = [&](const std::string & name) {
      const auto now = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(now - stamp).count();
      model.stage_ms.emplace_back(name, ms);
      stamp = now;
    };
  try {
    const auto ring = validatePolygon(boundary);
    mark("多边形校验");
    const BoundarySplit split = splitBoundary(ring, config);
    model.start_cap = split.start_cap;
    model.end_cap = split.end_cap;
    model.side_a.raw = split.side_a;
    model.side_b.raw = split.side_b;
    mark("端口与侧边");
    model.side_a = sampleSide(split.side_a, config);
    model.side_b = sampleSide(split.side_b, config);
    mark("两侧采样");
    model.valid = true;
    model.stage = "完成";
  } catch (const ModelError & error) {
    model.valid = false;
    model.stage = error.stage;
    model.reason = error.reason;
    model.side_a.samples.clear();
    model.side_b.samples.clear();
  }
  model.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return model;
}

}  // namespace garage_road_model
