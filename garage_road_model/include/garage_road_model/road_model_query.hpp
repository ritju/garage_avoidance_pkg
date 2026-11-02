#pragma once

#include "garage_road_model/types.hpp"

namespace garage_road_model
{

class RoadModelQuery
{
public:
  explicit RoadModelQuery(RoadModelData model);

  SideProjection nearestSide(const Eigen::Vector2d & point) const;

  // half_window < 0 时投影整条侧边。否则只投影锚点两侧的弧长窗口，避免跨到 U 形另一条臂。
  SideProjection projectOnSide(
    SideId side,
    const Eigen::Vector2d & point,
    double anchor_s,
    double half_window) const;

  SidePose evaluateSide(SideId side, double s) const;

  InwardNormal inwardNormal(SideId side, double s) const;

  CrossSection localCrossSection(SideId side, double s) const;

  const RoadModelData & model() const {return model_;}

private:
  const SideBoundary & boundaryOf(SideId side) const;

  RoadModelData model_;
};

}  // namespace garage_road_model
