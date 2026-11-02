#include "garage_road_model/road_model_builder.hpp"
#include "garage_road_model/road_model_query.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using garage_road_model::ModelConfig;
using garage_road_model::QueryStatus;
using garage_road_model::RoadModelBuilder;
using garage_road_model::RoadModelData;
using garage_road_model::RoadModelQuery;
using garage_road_model::SideId;

namespace
{

RoadModelData build(const std::vector<Eigen::Vector2d> & boundary)
{
  return RoadModelBuilder().build(boundary, 0, ModelConfig{});
}

double sideMeanY(const garage_road_model::SideBoundary & side)
{
  double sum = 0.0;
  for (const auto & sample : side.samples) {
    sum += sample.y;
  }
  return side.samples.empty() ? 0.0 : sum / static_cast<double>(side.samples.size());
}

bool hasCorner(const garage_road_model::SideBoundary & side)
{
  for (const auto & sample : side.samples) {
    if (sample.corner) {
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector2d> arcBand(double inner, double outer, double sweep, int steps)
{
  std::vector<Eigen::Vector2d> points;
  for (int i = 0; i <= steps; ++i) {
    const double angle = sweep * static_cast<double>(i) / static_cast<double>(steps);
    points.emplace_back(outer * std::cos(angle), outer * std::sin(angle));
  }
  for (int i = steps; i >= 0; --i) {
    const double angle = sweep * static_cast<double>(i) / static_cast<double>(steps);
    points.emplace_back(inner * std::cos(angle), inner * std::sin(angle));
  }
  return points;
}

}  // namespace

TEST(SideSplit, RectangleKeepsBothLongEdges)
{
  const auto model = build({{0, 0}, {20, 0}, {20, 2}, {0, 2}});
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_NEAR(model.side_a.length, 20.0, 0.05);
  EXPECT_NEAR(model.side_b.length, 20.0, 0.05);
  EXPECT_NEAR(std::min(sideMeanY(model.side_a), sideMeanY(model.side_b)), 0.0, 0.02);
  EXPECT_NEAR(std::max(sideMeanY(model.side_a), sideMeanY(model.side_b)), 2.0, 0.02);
  EXPECT_LT(model.side_a.max_error, 0.02);
  EXPECT_LT(model.side_b.max_error, 0.02);
  EXPECT_NEAR(model.start_cap.front().x(), model.start_cap.back().x(), 1e-6);

  RoadModelQuery query(model);
  const auto near_bottom = query.nearestSide(Eigen::Vector2d(10, 0.2));
  ASSERT_EQ(near_bottom.status, QueryStatus::Ok);
  EXPECT_EQ(near_bottom.side, SideId::A);
  const auto near_top = query.nearestSide(Eigen::Vector2d(10, 1.8));
  ASSERT_EQ(near_top.status, QueryStatus::Ok);
  EXPECT_EQ(near_top.side, SideId::B);
  const auto section = query.localCrossSection(SideId::A, 10.0);
  ASSERT_EQ(section.status, QueryStatus::Ok);
  EXPECT_NEAR(section.width, 2.0, 0.05);
}

TEST(SideSplit, CollinearPointsStayOnTheSameEdge)
{
  const auto model = build({{0, 0}, {5, 0}, {12, 0}, {20, 0}, {20, 2}, {0, 2}});
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_NEAR(model.side_a.length, 20.0, 0.05);
  EXPECT_NEAR(model.side_b.length, 20.0, 0.05);
  EXPECT_GE(model.side_a.raw.size(), 4U);
}

TEST(SideSplit, WindingAndStartIndexDoNotSwapDirection)
{
  const auto forward = build({{0, 0}, {20, 0}, {20, 2}, {0, 2}});
  const auto reversed = build({{0, 2}, {20, 2}, {20, 0}, {0, 0}});
  const auto rotated = build({{20, 0}, {20, 2}, {0, 2}, {0, 0}});
  ASSERT_TRUE(forward.valid && reversed.valid && rotated.valid);
  EXPECT_NEAR(forward.side_a.samples.front().x, reversed.side_a.samples.front().x, 1e-6);
  EXPECT_NEAR(forward.side_a.samples.front().yaw, reversed.side_a.samples.front().yaw, 1e-6);
  EXPECT_NEAR(forward.side_a.samples.front().x, rotated.side_a.samples.front().x, 1e-6);
  EXPECT_NEAR(forward.side_b.samples.front().y, reversed.side_b.samples.front().y, 1e-6);
}

TEST(SideSplit, BendKeepsTheCorner)
{
  const auto model = build({{0, 0}, {8, 0}, {8, 2}, {2, 2}, {2, 8}, {0, 8}});
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_GT(model.side_a.length, 10.0);
  EXPECT_GT(model.side_b.length, 10.0);
  EXPECT_TRUE(hasCorner(model.side_a));
  EXPECT_TRUE(hasCorner(model.side_b));
}

TEST(SideSplit, UShapeDoesNotBridgeTheGap)
{
  const auto model = build({{0, 0}, {10, 0}, {10, 8}, {8, 8}, {8, 2}, {2, 2}, {2, 8}, {0, 8}});
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_GT(std::min(model.side_a.length, model.side_b.length), 16.0);
  EXPECT_GT(std::max(model.side_a.length, model.side_b.length), 24.0);
  for (const auto & side : {model.side_a, model.side_b}) {
    for (const auto & sample : side.samples) {
      EXPECT_TRUE(sample.y < 2.2 || sample.x < 2.2 || sample.x > 7.8);
    }
  }
}

TEST(SideSplit, ArcKeepsInnerAndOuter)
{
  const auto model = build(arcBand(8.0, 10.0, M_PI_2, 16));
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_GT(model.side_a.length, 10.0);
  EXPECT_GT(model.side_b.length, 10.0);
  EXPECT_LT(std::min(model.side_a.length, model.side_b.length), 14.0);
  EXPECT_GT(std::max(model.side_a.length, model.side_b.length), 14.0);
}

TEST(SideSplit, SquareIsAmbiguous)
{
  const auto model = build({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
  EXPECT_FALSE(model.valid);
  EXPECT_NE(model.reason.find("歧义"), std::string::npos) << model.reason;
}

TEST(SideSplit, BowtieIsRejected)
{
  const auto model = build({{0, 0}, {2, 2}, {0, 2}, {2, 0}});
  EXPECT_FALSE(model.valid);
  EXPECT_NE(model.reason.find("自交"), std::string::npos) << model.reason;
}

TEST(SideSplit, SShapeKeepsTwoSides)
{
  const auto model = build({
    {0, 0}, {8, 0}, {8, 8}, {2, 8}, {2, 10}, {8, 10}, {8, 12}, {0, 12}, {0, 6}, {6, 6}, {6, 2}, {0, 2}});
  ASSERT_TRUE(model.valid) << model.stage << " " << model.reason;
  EXPECT_GT(model.side_a.length, 10.0);
  EXPECT_GT(model.side_b.length, 10.0);
}
