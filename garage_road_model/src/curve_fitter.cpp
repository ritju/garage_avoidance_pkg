#include "garage_road_model/curve_fitter.hpp"

#include "garage_road_model/polygon_processor.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace garage_road_model
{
namespace
{

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

double chordDeviation(const std::vector<Eigen::Vector2d> & piece)
{
  double worst = 0.0;
  for (const auto & point : piece) {
    worst = std::max(worst, distanceToEdges(point, {piece.front(), piece.back()}, false));
  }
  return worst;
}

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

}  // namespace garage_road_model
