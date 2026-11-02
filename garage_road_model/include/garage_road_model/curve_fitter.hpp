#pragma once

#include "garage_road_model/types.hpp"

#include <vector>

namespace garage_road_model
{

struct SplineCurve
{
  int degree{3};
  std::vector<double> knots;
  std::vector<Eigen::Vector2d> controls;

  Eigen::Vector2d evaluate(double u, int derivative = 0) const;
};

// 只拟合一条侧边。直段保持直线，大转角处分段，两端固定在原侧边端点上。
SideBoundary fitSideChain(
  const std::vector<Eigen::Vector2d> & chain,
  const ModelConfig & config);

}  // namespace garage_road_model
