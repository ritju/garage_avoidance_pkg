#pragma once

#include "garage_road_model/types.hpp"

namespace garage_road_model
{

class RoadModelQuery
{
public:
  // 保存一份道路模型，供后续查询。
  explicit RoadModelQuery(RoadModelData model);

  // 在两条侧边里找离点最近的投影。
  SideProjection nearestSide(const Eigen::Vector2d & point, double search_radius = -1.0) const;

  // half_window < 0 时投影整条侧边。否则只投影锚点两侧的弧长窗口，避免跨到 U 形另一条臂。
  SideProjection projectOnSide(
    SideId side,
    const Eigen::Vector2d & point,
    double anchor_s,
    double half_window) const;

  // 按弧长取侧边上的位置和切线。
  SidePose evaluateSide(SideId side, double s) const;

  // 取该弧长处朝通道内侧的法线。
  InwardNormal inwardNormal(SideId side, double s) const;

  // 取该弧长处的通道横截面。
  CrossSection localCrossSection(SideId side, double s) const;

  // 返回这份道路模型。
  const RoadModelData & model() const {return model_;}

private:
  // 按边号取出对应侧边。
  const SideBoundary & boundaryOf(SideId side) const;

  RoadModelData model_;
};

}  // namespace garage_road_model
