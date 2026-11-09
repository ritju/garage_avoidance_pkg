#include "garage_road_model/polygon_processor.hpp"
#include "garage_road_model/road_model_builder.hpp"
#include "garage_road_model/road_model_query.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace garage_road_model
{

struct SplineCurve
{
  int degree{3};
  std::vector<double> knots;
  std::vector<Eigen::Vector2d> controls;

  // 计算样条在参数 u 处的点。derivative 为导数阶数。
  Eigen::Vector2d evaluate(double u, int derivative = 0) const;
};

// 对一条长边做曲线拟合。
namespace
{

// 按弦长给每个点分配拟合参数。
std::vector<double> chordParameters(const std::vector<Eigen::Vector2d> & points)
{
  std::vector<double> u(points.size(), 0.0);
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
    u[i] = length;
  }
  if (length < 1e-6) {
    throw ModelError("曲线拟合", "侧边长度过短");
  }
  for (double & value : u) {
    value /= length;
  }
  u.front() = 0.0;
  u.back() = 1.0;
  return u;
}

// 生成两端夹紧的样条节点。
std::vector<double> clampedKnots(int controls, int degree)
{
  std::vector<double> knots(static_cast<size_t>(controls + degree + 1), 0.0);
  const int interior = controls - degree - 1;
  for (int i = 0; i < interior; ++i) {
    knots[static_cast<size_t>(degree + 1 + i)] =
      static_cast<double>(i + 1) / static_cast<double>(interior + 1);
  }
  for (int i = 0; i <= degree; ++i) {
    knots[static_cast<size_t>(controls + i)] = 1.0;
  }
  return knots;
}

// 查找参数落在哪个节点区间。
int findSpan(const SplineCurve & spline, double u)
{
  const int n = static_cast<int>(spline.controls.size());
  const int p = spline.degree;
  if (u >= spline.knots[static_cast<size_t>(n)]) {
    return n - 1;
  }
  for (int i = p; i < n; ++i) {
    if (u < spline.knots[static_cast<size_t>(i + 1)]) {
      return i;
    }
  }
  return n - 1;
}

// 用德布尔算法计算样条上的点。
Eigen::Vector2d deBoor(const SplineCurve & spline, double u)
{
  const int p = spline.degree;
  const int span = findSpan(spline, std::clamp(u, 0.0, 1.0));
  std::vector<Eigen::Vector2d> local(static_cast<size_t>(p + 1));
  for (int j = 0; j <= p; ++j) {
    local[static_cast<size_t>(j)] = spline.controls[static_cast<size_t>(span - p + j)];
  }
  for (int r = 1; r <= p; ++r) {
    for (int j = p; j >= r; --j) {
      const int index = span - p + j;
      const double left = spline.knots[static_cast<size_t>(index)];
      const double right = spline.knots[static_cast<size_t>(index + p - r + 1)];
      const double denom = right - left;
      const double alpha = std::abs(denom) < 1e-15 ? 0.0 : (u - left) / denom;
      local[static_cast<size_t>(j)] =
        (1.0 - alpha) * local[static_cast<size_t>(j - 1)] + alpha * local[static_cast<size_t>(j)];
    }
  }
  return local[static_cast<size_t>(p)];
}

// 求样条的导数曲线。
SplineCurve derivativeOf(const SplineCurve & spline)
{
  SplineCurve derived;
  derived.degree = spline.degree - 1;
  if (spline.knots.size() < 2 || spline.controls.size() < 2) {
    throw ModelError("曲线采样", "导数退化");
  }
  derived.knots.assign(spline.knots.begin() + 1, spline.knots.end() - 1);
  derived.controls.reserve(spline.controls.size() - 1);
  for (size_t i = 0; i + 1 < spline.controls.size(); ++i) {
    const double denom = spline.knots[i + static_cast<size_t>(spline.degree) + 1] - spline.knots[i + 1];
    const double scale = std::abs(denom) > 1e-12 ? static_cast<double>(spline.degree) / denom : 0.0;
    derived.controls.push_back(scale * (spline.controls[i + 1] - spline.controls[i]));
  }
  return derived;
}

// 固定两端控制点，求解中间控制点。
bool solveInterior(
  const std::vector<Eigen::Vector2d> & samples,
  const std::vector<double> & parameters,
  int control_count,
  double lambda,
  SplineCurve & curve)
{
  const int degree = 3;
  if (control_count < degree + 1 || samples.size() < 4) {
    return false;
  }
  curve.degree = degree;
  curve.knots = clampedKnots(control_count, degree);
  curve.controls.assign(static_cast<size_t>(control_count), Eigen::Vector2d::Zero());
  curve.controls.front() = samples.front();
  curve.controls.back() = samples.back();

  const int interior = control_count - 2;
  if (interior <= 0) {
    return false;
  }
  const int rows = static_cast<int>(samples.size()) + std::max(0, control_count - 2);
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(rows, interior);
  Eigen::VectorXd rhs_x = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd rhs_y = Eigen::VectorXd::Zero(rows);

  SplineCurve probe = curve;
  for (int i = 0; i < interior; ++i) {
    probe.controls[static_cast<size_t>(i + 1)] = Eigen::Vector2d::Zero();
  }

  for (int row = 0; row < static_cast<int>(samples.size()); ++row) {
    const double u = parameters[static_cast<size_t>(row)];
    Eigen::Vector2d known = Eigen::Vector2d::Zero();
    for (int col = 0; col < control_count; ++col) {
      probe.controls[static_cast<size_t>(col)] =
        (col == 0 || col == control_count - 1) ? curve.controls[static_cast<size_t>(col)] :
        Eigen::Vector2d::UnitX();
    }
    // Basis values by evaluating unit impulses.
    for (int col = 0; col < control_count; ++col) {
      for (auto & control : probe.controls) {
        control.setZero();
      }
      probe.controls[static_cast<size_t>(col)] = Eigen::Vector2d::UnitX();
      const double basis = deBoor(probe, u).x();
      if (col == 0 || col == control_count - 1) {
        known += basis * curve.controls[static_cast<size_t>(col)];
      } else {
        matrix(row, col - 1) = basis;
      }
    }
    const Eigen::Vector2d residual = samples[static_cast<size_t>(row)] - known;
    rhs_x(row) = residual.x();
    rhs_y(row) = residual.y();
  }

  const double weight = std::sqrt(std::max(lambda, 0.0));
  int smooth_row = static_cast<int>(samples.size());
  for (int k = 0; k <= control_count - 3 && smooth_row < rows; ++k, ++smooth_row) {
    auto add = [&](int control, double coefficient) {
        if (control == 0) {
          rhs_x(smooth_row) -= weight * coefficient * curve.controls.front().x();
          rhs_y(smooth_row) -= weight * coefficient * curve.controls.front().y();
        } else if (control == control_count - 1) {
          rhs_x(smooth_row) -= weight * coefficient * curve.controls.back().x();
          rhs_y(smooth_row) -= weight * coefficient * curve.controls.back().y();
        } else {
          matrix(smooth_row, control - 1) += weight * coefficient;
        }
      };
    add(k, 1.0);
    add(k + 1, -2.0);
    add(k + 2, 1.0);
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(matrix);
  if (solver.rank() < interior) {
    return false;
  }
  const Eigen::VectorXd solution_x = solver.solve(rhs_x);
  const Eigen::VectorXd solution_y = solver.solve(rhs_y);
  if (!solution_x.allFinite() || !solution_y.allFinite()) {
    return false;
  }
  for (int i = 0; i < interior; ++i) {
    curve.controls[static_cast<size_t>(i + 1)] =
      Eigen::Vector2d(solution_x(i), solution_y(i));
  }
  return true;
}

// 按弧长在样条上采样。
std::vector<RoadSampleData> sampleByArcLength(const SplineCurve & spline, double spacing)
{
  constexpr int kDense = 400;
  std::vector<double> parameter(kDense);
  std::vector<Eigen::Vector2d> positions(kDense);
  std::vector<double> cumulative(kDense, 0.0);
  for (int i = 0; i < kDense; ++i) {
    parameter[static_cast<size_t>(i)] = static_cast<double>(i) / static_cast<double>(kDense - 1);
    positions[static_cast<size_t>(i)] = spline.evaluate(parameter[static_cast<size_t>(i)]);
    if (i > 0) {
      cumulative[static_cast<size_t>(i)] = cumulative[static_cast<size_t>(i - 1)] +
        (positions[static_cast<size_t>(i)] - positions[static_cast<size_t>(i - 1)]).norm();
    }
  }
  const double total = cumulative.back();
  if (total < 1e-4) {
    throw ModelError("曲线采样", "弧长过短");
  }
  const int count = std::max(2, static_cast<int>(std::ceil(total / spacing)) + 1);
  std::vector<RoadSampleData> samples;
  samples.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double target = total * static_cast<double>(i) / static_cast<double>(count - 1);
    auto it = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    int upper = static_cast<int>(std::distance(cumulative.begin(), it));
    upper = std::clamp(upper, 1, kDense - 1);
    const int lower = upper - 1;
    const double span = cumulative[static_cast<size_t>(upper)] - cumulative[static_cast<size_t>(lower)];
    const double ratio = span < 1e-12 ? 0.0 :
      (target - cumulative[static_cast<size_t>(lower)]) / span;
    const double u = parameter[static_cast<size_t>(lower)] +
      ratio * (parameter[static_cast<size_t>(upper)] - parameter[static_cast<size_t>(lower)]);
    const Eigen::Vector2d position = spline.evaluate(u);
    const Eigen::Vector2d first = spline.evaluate(u, 1);
    const Eigen::Vector2d second = spline.evaluate(u, 2);
    const double speed_sq = first.squaredNorm();
    if (speed_sq < 1e-10) {
      throw ModelError("曲线采样", "导数退化");
    }
    const double speed = std::sqrt(speed_sq);
    const double curvature =
      (first.x() * second.y() - first.y() * second.x()) / (speed_sq * speed);
    if (!std::isfinite(curvature)) {
      throw ModelError("曲线采样", "曲率不是有限值");
    }
    RoadSampleData sample;
    sample.x = position.x();
    sample.y = position.y();
    sample.yaw = std::atan2(first.y(), first.x());
    sample.s = target;
    sample.curvature = curvature;
    samples.push_back(sample);
  }
  for (size_t i = 1; i < samples.size(); ++i) {
    if (!(samples[i].s > samples[i - 1].s)) {
      throw ModelError("曲线采样", "弧长没有严格递增");
    }
  }
  return samples;
}

// 按边长估计这一段需要多少控制点。
int initialControls(const std::vector<Eigen::Vector2d> & samples, const ModelConfig & config)
{
  double length = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    length += (samples[i] - samples[i - 1]).norm();
  }
  const int estimated = static_cast<int>(std::ceil(length / config.control_spacing)) + 4;
  return std::clamp(estimated, 4, config.max_controls);
}

}  // namespace

// 计算样条在参数处的点，导数阶数由 derivative 指定。
Eigen::Vector2d SplineCurve::evaluate(double u, int derivative) const
{
  if (controls.empty() || knots.empty()) {
    throw ModelError("曲线采样", "导数退化");
  }
  const double clamped = std::clamp(u, 0.0, 1.0);
  if (derivative <= 0) {
    return deBoor(*this, clamped);
  }
  if (derivative > degree) {
    return Eigen::Vector2d::Zero();
  }
  SplineCurve derived = *this;
  for (int order = 0; order < derivative; ++order) {
    derived = derivativeOf(derived);
  }
  return deBoor(derived, clamped);
}

// 计算三个点之间的转角。
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

// 点列偏离两端连线的最大距离。
double chordDeviation(const std::vector<Eigen::Vector2d> & piece)
{
  double worst = 0.0;
  for (const auto & point : piece) {
    worst = std::max(worst, distanceToEdges(point, {piece.front(), piece.back()}, false));
  }
  return worst;
}

// 给采样点的弧长加上偏移。
void shiftSamples(std::vector<RoadSampleData> & samples, double offset, bool mark_corner)
{
  if (samples.empty()) {
    return;
  }
  for (auto & sample : samples) {
    sample.s += offset;
  }
  if (mark_corner) {
    samples.front().corner = true;
  }
}

// 把一段直线按间距采样。
std::vector<RoadSampleData> sampleStraight(const std::vector<Eigen::Vector2d> & piece, double spacing)
{
  const Eigen::Vector2d delta = piece.back() - piece.front();
  const double length = delta.norm();
  if (length < 1e-4) {
    throw ModelError("曲线拟合", "侧边长度过短");
  }
  const double yaw = std::atan2(delta.y(), delta.x());
  const int count = std::max(2, static_cast<int>(std::ceil(length / spacing)) + 1);
  std::vector<RoadSampleData> samples;
  samples.reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count - 1);
    RoadSampleData sample;
    const Eigen::Vector2d point = piece.front() + ratio * delta;
    sample.x = point.x();
    sample.y = point.y();
    sample.yaw = yaw;
    sample.s = ratio * length;
    sample.curvature = 0.0;
    samples.push_back(sample);
  }
  return samples;
}

// 把一段折线按间距采样。
std::vector<RoadSampleData> samplePolyline(const std::vector<Eigen::Vector2d> & piece, double spacing)
{
  const auto dense = resamplePolyline(piece, spacing);
  if (dense.size() < 2) {
    throw ModelError("曲线拟合", "侧边长度过短");
  }
  std::vector<RoadSampleData> samples;
  double traveled = 0.0;
  for (size_t i = 0; i < dense.size(); ++i) {
    Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
    if (i + 1 < dense.size()) {
      direction = dense[i + 1] - dense[i];
    } else {
      direction = dense[i] - dense[i - 1];
    }
    if (i > 0) {
      traveled += (dense[i] - dense[i - 1]).norm();
    }
    RoadSampleData sample;
    sample.x = dense[i].x();
    sample.y = dense[i].y();
    sample.yaw = std::atan2(direction.y(), direction.x());
    sample.s = traveled;
    sample.curvature = 0.0;
    samples.push_back(sample);
  }
  return samples;
}

// 计算拟合点到原折线的误差。
void measureError(
  const std::vector<RoadSampleData> & samples,
  const std::vector<Eigen::Vector2d> & source,
  double & max_error,
  double & mean_error)
{
  std::vector<Eigen::Vector2d> fitted;
  fitted.reserve(samples.size());
  double sum = 0.0;
  int count = 0;
  max_error = 0.0;
  auto accumulate = [&](double distance) {
      max_error = std::max(max_error, distance);
      sum += distance;
      ++count;
    };
  for (const auto & sample : samples) {
    const Eigen::Vector2d point(sample.x, sample.y);
    fitted.push_back(point);
    accumulate(distanceToEdges(point, source, false));
  }
  for (const auto & point : source) {
    accumulate(distanceToEdges(point, fitted, false));
  }
  mean_error = count == 0 ? 0.0 : sum / static_cast<double>(count);
}

// 拟合侧边上的一段。
std::vector<RoadSampleData> fitPiece(
  const std::vector<Eigen::Vector2d> & piece,
  const ModelConfig & config)
{
  if (piece.size() < 2) {
    throw ModelError("曲线拟合", "侧边点数不足");
  }
  if (chordDeviation(piece) <= config.collinear_tolerance) {
    return sampleStraight(piece, config.sample_spacing);
  }
  const auto dense = resamplePolyline(piece, config.resample_spacing);
  if (dense.size() < 4) {
    return samplePolyline(piece, config.sample_spacing);
  }
  const auto parameters = chordParameters(dense);
  int controls = std::min(initialControls(dense, config), static_cast<int>(dense.size()));
  double lambda = config.initial_lambda;
  std::string last_reason = "拟合误差过大";
  for (int attempt = 0; attempt < config.fit_attempts; ++attempt) {
    SplineCurve curve;
    if (!solveInterior(dense, parameters, controls, lambda, curve)) {
      last_reason = "最小二乘求解失败";
      controls = std::max(4, controls - 1);
      continue;
    }
    std::vector<RoadSampleData> samples;
    try {
      samples = sampleByArcLength(curve, config.sample_spacing);
    } catch (const ModelError & error) {
      last_reason = error.reason;
      lambda *= 0.25;
      continue;
    }
    double max_error = 0.0;
    double mean_error = 0.0;
    measureError(samples, piece, max_error, mean_error);
    if (max_error <= config.max_fit_error) {
      return samples;
    }
    last_reason = "拟合偏离原侧边";
    lambda *= 0.25;
    controls = std::min(std::min(config.max_controls, static_cast<int>(dense.size())), controls + 2);
  }
  throw ModelError("曲线拟合", last_reason);
}

// 拟合整条侧边。直段保持直线，转角过大就分段。
SideBoundary fitSideChain(
  const std::vector<Eigen::Vector2d> & chain,
  const ModelConfig & config)
{
  if (chain.size() < 2) {
    throw ModelError("曲线拟合", "侧边点数不足");
  }
  std::vector<std::vector<Eigen::Vector2d>> pieces;
  std::vector<Eigen::Vector2d> current{chain.front()};
  for (size_t i = 1; i + 1 < chain.size(); ++i) {
    current.push_back(chain[i]);
    if (pieceTurn(chain[i - 1], chain[i], chain[i + 1]) >= config.piece_angle_rad) {
      pieces.push_back(current);
      current = {chain[i]};
    }
  }
  current.push_back(chain.back());
  pieces.push_back(std::move(current));

  SideBoundary side;
  side.raw = chain;
  for (size_t i = 0; i < pieces.size(); ++i) {
    auto samples = fitPiece(pieces[i], config);
    const double offset = side.samples.empty() ? 0.0 : side.samples.back().s;
    const bool join = !side.samples.empty();
    if (join && !samples.empty() &&
      std::hypot(samples.front().x - side.samples.back().x, samples.front().y - side.samples.back().y) < 1e-6)
    {
      side.samples.back().corner = true;
      samples.erase(samples.begin());
    }
    shiftSamples(samples, offset, join && !samples.empty() && !side.samples.back().corner);
    side.samples.insert(side.samples.end(), samples.begin(), samples.end());
  }
  if (side.samples.size() < 2) {
    throw ModelError("曲线拟合", "侧边采样失败");
  }
  side.length = side.samples.back().s;
  measureError(side.samples, side.raw, side.max_error, side.mean_error);
  if (side.max_error > config.max_fit_error) {
    throw ModelError("曲线拟合", "拟合偏离原侧边");
  }
  return side;
}


// 检查拟合后的两条边有没有交叉。
bool rejectFittedSides(
  const SideBoundary & side_a,
  const SideBoundary & side_b,
  std::string & reason)
{
  // 取出一条侧边的采样点坐标。
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


// 一条通道的拟合流程。
namespace
{
// 按输出间距抽稀，保留拐角；抽稀后走样就退回密集点。
void sampleOutput(SideBoundary & side, double spacing, double tolerance)
{
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw ModelError("输出采样", "output_sample_spacing 必须是有限正数");
  }
  const auto original = side.samples;
  if (original.size() < 2) {return;}
  const double length = original.back().s;
  if (length / spacing > 1000000.0) {
    throw ModelError("输出采样", "输出点数量超过上限");
  }
  std::vector<double> targets{original.front().s, length};
  for (size_t k = 1; static_cast<double>(k) * spacing < length; ++k) {
    targets.push_back(static_cast<double>(k) * spacing);
  }
  for (const auto & point : original) {
    if (point.corner) {targets.push_back(point.s);}
  }
  std::sort(targets.begin(), targets.end());
  targets.erase(std::unique(targets.begin(), targets.end()), targets.end());
  std::vector<RoadSampleData> output;
  for (double s : targets) {
    auto upper = std::lower_bound(original.begin(), original.end(), s,
      [](const RoadSampleData & point, double value) {return point.s < value;});
    RoadSampleData point;
    if (upper == original.begin() || upper == original.end() || upper->s == s) {
      point = upper == original.end() ? original.back() : *upper;
    } else {
      const auto & a = *(upper - 1);
      const auto & b = *upper;
      const double t = (s - a.s) / (b.s - a.s);
      point.x = a.x + t * (b.x - a.x);
      point.y = a.y + t * (b.y - a.y);
      point.s = s;
      point.yaw = a.yaw + t * std::atan2(std::sin(b.yaw-a.yaw), std::cos(b.yaw-a.yaw));
      point.curvature = a.curvature + t * (b.curvature-a.curvature);
    }
    if (!output.empty()) {
      const auto start = output.back();
      const Eigen::Vector2d a(start.x, start.y), b(point.x, point.y);
      const auto delta = b-a;
      bool preserve = false;
      auto begin = std::upper_bound(original.begin(), original.end(), start.s,
        [](double value, const RoadSampleData & item) {return value < item.s;});
      for (auto it = begin; it != original.end() && it->s < s; ++it) {
        const Eigen::Vector2d v(it->x, it->y);
        const double t = delta.squaredNorm() > 1e-20 ?
          std::clamp((v-a).dot(delta)/delta.squaredNorm(), 0.0, 1.0) : 0.0;
        if ((v-a-t*delta).norm() > tolerance) {preserve = true; break;}
      }
      if (preserve) {
        for (auto it = begin; it != original.end() && it->s < s; ++it) {
          output.push_back(*it);
        }
      }
    }
    output.push_back(point);
  }
  side.samples = std::move(output);
}
}  // namespace

// 校验边界、切出长边、拟合两侧并输出采样。
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
    model.boundary = validatePolygon(boundary);
    mark("多边形校验");
    const BoundarySplit split = splitBoundary(model.boundary, config);
    model.boundary = split.boundary;
    model.start_cap = split.start_cap;
    model.end_cap = split.end_cap;
    model.side_a.raw = split.side_a;
    model.side_b.raw = split.side_b;
    mark("端口与侧边");
    model.side_a = fitSideChain(split.side_a, config);
    model.side_b = fitSideChain(split.side_b, config);
    mark("两侧拟合");
    std::string cross_reason;
    if (rejectFittedSides(model.side_a, model.side_b, cross_reason)) {
      throw ModelError("曲线校验", cross_reason);
    }
    mark("两侧校验");
    const auto dense_a = model.side_a;
    const auto dense_b = model.side_b;
    sampleOutput(model.side_a, config.output_sample_spacing, config.max_fit_error);
    sampleOutput(model.side_b, config.output_sample_spacing, config.max_fit_error);
    if (rejectFittedSides(model.side_a, model.side_b, cross_reason)) {
      model.side_a = dense_a;
      model.side_b = dense_b;
    }
    mark("输出采样");
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


// 拟合完成后，按弧长查询投影、切线和朝内法线。
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

// 取采样点的平面坐标。
Eigen::Vector2d positionOf(const RoadSampleData & sample)
{
  return Eigen::Vector2d(sample.x, sample.y);
}

// 把角度折到 [-π, π]。
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

// 把点投到一段采样折线上。
SegmentHit projectOnto(
  const Eigen::Vector2d & point,
  const std::vector<RoadSampleData> & samples,
  double anchor_s,
  double half_window,
  double search_radius = -1.0)
{
  SegmentHit best;
  SegmentHit second;
  if (samples.size() < 2) {
    return best;
  }
  for (size_t i = 0; i + 1 < samples.size(); ++i) {
    if (half_window >= 0.0) {
      const double s0 = samples[i].s;
      const double s1 = samples[i + 1].s;
      if (s1 < anchor_s - half_window || s0 > anchor_s + half_window) {
        continue;
      }
    }
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
    if (search_radius >= 0.0 && distance > search_radius) {continue;}
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

// 保存一份道路模型，供后续查询。
RoadModelQuery::RoadModelQuery(RoadModelData model)
: model_(std::move(model))
{
}

// 按边号取出对应侧边。
const SideBoundary & RoadModelQuery::boundaryOf(SideId side) const
{
  return side == SideId::A ? model_.side_a : model_.side_b;
}

// 在两条侧边里找离点最近的投影。
SideProjection RoadModelQuery::nearestSide(const Eigen::Vector2d & point, double search_radius) const
{
  SideProjection result;
  if (!model_.valid) {
    result.reason = "道路无效";
    return result;
  }
  const SegmentHit side_a = projectOnto(point, model_.side_a.samples, 0.0, -1.0, search_radius);
  const SegmentHit side_b = projectOnto(point, model_.side_b.samples, 0.0, -1.0, search_radius);
  if (!std::isfinite(side_a.distance) && !std::isfinite(side_b.distance)) {
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

// 把点投影到指定侧边。
SideProjection RoadModelQuery::projectOnSide(
  SideId side,
  const Eigen::Vector2d & point,
  double anchor_s,
  double half_window) const
{
  SideProjection result;
  result.side = side;
  if (!model_.valid) {
    result.reason = "道路无效";
    return result;
  }
  const SegmentHit hit = projectOnto(point, boundaryOf(side).samples, anchor_s, half_window);
  if (!std::isfinite(hit.distance)) {
    result.reason = half_window >= 0.0 ? "弧长窗口内没有侧边" : "侧边无法投影";
    return result;
  }
  result.s = hit.s;
  result.point = hit.point;
  result.distance = hit.distance;
  result.yaw = hit.yaw;
  if (hit.gap_s > 1.0) {
    result.status = QueryStatus::Ambiguous;
    result.reason = "同一侧边有多个远隔投影";
    return result;
  }
  result.status = QueryStatus::Ok;
  return result;
}

// 按弧长取侧边上的位置和切线。
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

// 取该弧长处朝通道内侧的法线。
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
  if (left.squaredNorm() < 0.5) {
    result.reason = "切线退化";
    return result;
  }
  constexpr double kProbe = 0.15;
  const bool left_inside = pointInsidePolygon(
    pose.position + kProbe * left, model_.boundary, 0.02);
  const bool right_inside = pointInsidePolygon(
    pose.position - kProbe * left, model_.boundary, 0.02);
  if (left_inside == right_inside) {
    result.status = QueryStatus::Ambiguous;
    result.reason = "朝内方向不明确";
    return result;
  }
  result.direction = left_inside ? left : Eigen::Vector2d(-left.x(), -left.y());
  result.status = QueryStatus::Ok;
  return result;
}

// 取该弧长处的通道横截面。
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
  std::vector<double> crossings;
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
    if (distance >= -0.2 && ratio >= -1e-6 && ratio <= 1.0 + 1e-6) {
      crossings.push_back(distance);
    }
  }
  std::sort(crossings.begin(), crossings.end());
  crossings.erase(
    std::unique(crossings.begin(), crossings.end(), [](double lhs, double rhs) {
      return std::abs(lhs - rhs) < 1e-5;
    }),
    crossings.end());
  double near_t = 0.0;
  double far_t = 0.0;
  bool found = false;
  for (size_t i = 0; i + 1 < crossings.size(); ++i) {
    const double t0 = crossings[i];
    const double t1 = crossings[i + 1];
    if (t0 > 0.12 || t1 <= t0 + 1e-4) {
      continue;
    }
    const Eigen::Vector2d mid = pose.position + (0.5 * (t0 + t1)) * normal.direction;
    if (!pointInsidePolygon(mid, polygon, 0.0)) {
      continue;
    }
    if (!found || std::abs(t0) < std::abs(near_t)) {
      near_t = t0;
      far_t = t1;
      found = true;
    }
  }
  if (!found) {
    section.reason = "没有朝内的有效区间";
    return section;
  }
  const Eigen::Vector2d near = pose.position + near_t * normal.direction;
  const Eigen::Vector2d far = pose.position + far_t * normal.direction;
  const auto & this_side = boundaryOf(side).raw;
  const auto & other_side = boundaryOf(side == SideId::A ? SideId::B : SideId::A).raw;
  if (distanceToEdges(far, this_side, false) < 0.15) {
    section.status = QueryStatus::Ambiguous;
    section.reason = "横截面打到同侧回折";
    return section;
  }
  const double cap_distance = std::min(
    distanceToEdges(far, model_.start_cap, false),
    distanceToEdges(far, model_.end_cap, false));
  const double other_distance = distanceToEdges(far, other_side, false);
  if (cap_distance + 1e-3 < other_distance && cap_distance < 0.25) {
    section.status = QueryStatus::Ambiguous;
    section.reason = "横截面打到端口";
    return section;
  }
  section.status = QueryStatus::Ok;
  section.s = s;
  section.width = far_t - near_t;
  section.near_point = near;
  section.far_point = far;
  return section;
}

}  // namespace garage_road_model
