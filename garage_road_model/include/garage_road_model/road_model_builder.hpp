#pragma once

#include "garage_road_model/types.hpp"

#include <vector>

namespace garage_road_model
{

class RoadModelBuilder
{
public:
  // 校验边界、切出长边、拟合两侧并输出采样。
  RoadModelData build(
    const std::vector<Eigen::Vector2d> & boundary,
    uint32_t source_index,
    const ModelConfig & config) const;
};

}  // namespace garage_road_model
