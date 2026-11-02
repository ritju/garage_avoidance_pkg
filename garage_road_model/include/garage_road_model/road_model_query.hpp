#pragma once

#include "garage_road_model/types.hpp"

namespace garage_road_model
{

class RoadModelQuery
{
public:
  explicit RoadModelQuery(RoadModelData model);

  SideProjection nearestSide(const Eigen::Vector2d & point) const;

  SidePose evaluateSide(SideId side, double s) const;

  InwardNormal inwardNormal(SideId side, double s) const;

  CrossSection localCrossSection(SideId side, double s) const;

  const RoadModelData & model() const {return model_;}

private:
  const SideBoundary & boundaryOf(SideId side) const;

  RoadModelData model_;
};

}  // namespace garage_road_model
