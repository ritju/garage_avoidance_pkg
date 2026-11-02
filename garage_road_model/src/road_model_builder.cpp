#include "garage_road_model/road_model_builder.hpp"

#include "garage_road_model/boundary_splitter.hpp"
#include "garage_road_model/curve_fitter.hpp"
#include "garage_road_model/curve_validator.hpp"
#include "garage_road_model/polygon_processor.hpp"

#include <chrono>

namespace garage_road_model
{

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
