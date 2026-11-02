#pragma once

#include "garage_road_model/types.hpp"

#include <string>
#include <vector>

namespace garage_road_model
{

// 两条拟合侧边不得自交，也不得互相穿越。
bool rejectFittedSides(
  const SideBoundary & side_a,
  const SideBoundary & side_b,
  std::string & reason);

}  // namespace garage_road_model
