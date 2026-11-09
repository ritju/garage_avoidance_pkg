#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace garage_road_model
{

struct ModelConfig
{
  // 共线合并：点到弦的距离不超过该值（米）才视为同一条直边。
  double collinear_tolerance{0.05};
  // 简化边界上，转角达到该值（弧度）才作为端头/侧边的分界。
  double corner_angle_rad{0.9};
  // 端头长度必须不超过较短侧边的这个比例，近正方形因此无法通过。
  double cap_side_ratio{0.75};
  // 端头与侧边相接处的转角下限（弧度）。
  double junction_angle_rad{0.7};
  // 沿侧边采样时，朝对侧打中且连线在多边形内的比例下限。
  double width_success_fraction{0.45};
  // 侧边内部超过该转角（弧度）就分段，避免把真实拐角拟合成圆角。
  double piece_angle_rad{0.5};
  double resample_spacing{0.2};
  double sample_spacing{0.2};
  double output_sample_spacing{0.5};
  double max_fit_error{0.08};
  int fit_attempts{5};
  double initial_lambda{1.0};
  int max_controls{48};
  double control_spacing{1.2};
  double curve_check_spacing{0.05};
  std::string frame_id{"map"};
};

struct RoadSampleData
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double s{0.0};
  double curvature{0.0};
  bool corner{false};
};

struct SideBoundary
{
  std::vector<Eigen::Vector2d> raw;
  std::vector<RoadSampleData> samples;
  double length{0.0};
  double max_error{0.0};
  double mean_error{0.0};
};

struct RoadModelData
{
  uint32_t source_index{0};
  std::vector<Eigen::Vector2d> boundary;
  std::vector<Eigen::Vector2d> start_cap;
  std::vector<Eigen::Vector2d> end_cap;
  SideBoundary side_a;
  SideBoundary side_b;
  bool valid{false};
  std::string reason;
  std::string stage;
  double elapsed_ms{0.0};
  std::vector<std::pair<std::string, double>> stage_ms;
};

enum class SideId
{
  A = 0,
  B = 1
};

enum class QueryStatus
{
  Ok,
  Ambiguous,
  OutOfRange,
  Invalid
};

struct SideProjection
{
  QueryStatus status{QueryStatus::Invalid};
  SideId side{SideId::A};
  double s{0.0};
  Eigen::Vector2d point{Eigen::Vector2d::Zero()};
  double distance{0.0};
  double yaw{0.0};
  std::string reason;
};

struct SidePose
{
  QueryStatus status{QueryStatus::Invalid};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Vector2d tangent{Eigen::Vector2d::Zero()};
  double yaw{0.0};
  double s{0.0};
  bool corner{false};
  std::string reason;
};

struct InwardNormal
{
  QueryStatus status{QueryStatus::Invalid};
  Eigen::Vector2d direction{Eigen::Vector2d::Zero()};
  std::string reason;
};

struct CrossSection
{
  QueryStatus status{QueryStatus::Invalid};
  double s{0.0};
  double width{0.0};
  Eigen::Vector2d near_point{Eigen::Vector2d::Zero()};
  Eigen::Vector2d far_point{Eigen::Vector2d::Zero()};
  std::string reason;
};

inline bool resultIsCurrent(uint64_t seq, uint64_t latest, bool newer_pending)
{
  return seq == latest && !newer_pending;
}

class ModelError : public std::runtime_error
{
public:
  ModelError(std::string stage_in, std::string reason_in)
  : std::runtime_error(stage_in + ": " + reason_in),
    stage(std::move(stage_in)),
    reason(std::move(reason_in))
  {
  }

  std::string stage;
  std::string reason;
};

}  // namespace garage_road_model
