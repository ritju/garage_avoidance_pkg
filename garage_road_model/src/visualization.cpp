#include "garage_road_model/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace garage_road_model
{

// 把端口和两条采样长边画成标记。通道外框用 garage_polygons，这里不再画一份。
visualization_msgs::msg::MarkerArray makeMarkers(
  const std::vector<RoadModelData> & roads,
  const std::string & frame_id,
  int64_t stamp_sec,
  uint32_t stamp_nanosec)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker wipe;
  wipe.header.frame_id = frame_id;
  wipe.header.stamp.sec = static_cast<int32_t>(stamp_sec);
  wipe.header.stamp.nanosec = stamp_nanosec;
  wipe.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(wipe);

  auto stamp = wipe.header;
  int id = 0;
  // 创建一个指定颜色和尺寸的标记。
  auto add = [&](const std::string & ns, int type, float r, float g, float b, float scale) {
      visualization_msgs::msg::Marker marker;
      marker.header = stamp;
      marker.ns = ns;
      marker.id = id++;
      marker.type = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 1.0;
      marker.pose.orientation.w = 1.0;
      return marker;
    };

  for (const auto & road : roads) {
    // 把一组点画成折线或点。
    auto drawChain = [&](const std::string & ns, const std::vector<Eigen::Vector2d> & points,
      int type, float r, float g, float b, float scale) {
        auto marker = add(ns, type, r, g, b, scale);
        for (const auto & point : points) {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          marker.points.push_back(p);
        }
        if (marker.points.size() >= 2) {
          array.markers.push_back(marker);
        }
      };
    drawChain("start_cap", road.start_cap, visualization_msgs::msg::Marker::LINE_STRIP, 0.95F, 0.55F, 0.1F, 0.08F);
    drawChain("end_cap", road.end_cap, visualization_msgs::msg::Marker::LINE_STRIP, 0.55F, 0.2F, 0.75F, 0.08F);
    drawChain("side_a_raw", road.side_a.raw, visualization_msgs::msg::Marker::POINTS, 0.9F, 0.45F, 0.4F, 0.06F);
    drawChain("side_b_raw", road.side_b.raw, visualization_msgs::msg::Marker::POINTS, 0.4F, 0.75F, 0.45F, 0.06F);

    // 画出采样折线，并标出几个切线方向。
    auto drawFit = [&](const std::string & ns, const SideBoundary & side, float r, float g, float b) {
        auto curve = add(ns, visualization_msgs::msg::Marker::LINE_STRIP, r, g, b, 0.05F);
        for (const auto & sample : side.samples) {
          geometry_msgs::msg::Point p;
          p.x = sample.x;
          p.y = sample.y;
          curve.points.push_back(p);
        }
        if (curve.points.size() >= 2) {
          array.markers.push_back(curve);
        }
        const int arrow_count = std::min(4, static_cast<int>(side.samples.size()));
        for (int arrow = 0; arrow < arrow_count; ++arrow) {
          const auto & sample = side.samples[
            side.samples.size() * static_cast<size_t>(arrow) / static_cast<size_t>(std::max(arrow_count, 1))];
          auto marker = add(ns + "_heading", visualization_msgs::msg::Marker::ARROW, r, g, b, 0.05F);
          marker.scale.y = 0.1F;
          marker.scale.z = 0.1F;
          geometry_msgs::msg::Point start;
          start.x = sample.x;
          start.y = sample.y;
          geometry_msgs::msg::Point end;
          end.x = sample.x + 0.45 * std::cos(sample.yaw);
          end.y = sample.y + 0.45 * std::sin(sample.yaw);
          marker.points.push_back(start);
          marker.points.push_back(end);
          array.markers.push_back(marker);
        }
      };
    drawFit("side_a", road.side_a, 0.85F, 0.15F, 0.1F);
    drawFit("side_b", road.side_b, 0.1F, 0.55F, 0.25F);
  }
  return array;
}

}  // namespace garage_road_model
