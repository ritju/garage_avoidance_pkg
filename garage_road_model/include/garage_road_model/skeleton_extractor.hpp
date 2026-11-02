#pragma once

#include "garage_road_model/types.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace garage_road_model
{

struct TrunkExtraction
{
  std::vector<Eigen::Vector2d> trunk;
  std::vector<Eigen::Vector2d> skeleton;
};

TrunkExtraction extractTrunk(
  const cv::Mat & mask,
  const GridGeometry & grid,
  const std::vector<Eigen::Vector2d> & polygon,
  const ModelConfig & config);

}  // namespace garage_road_model
