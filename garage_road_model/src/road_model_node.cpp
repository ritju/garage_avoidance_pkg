#include "garage_road_model/road_model_builder.hpp"

#include <garage_road_model/msg/road_model_array.hpp>
#include <garage_utils_msgs/msg/polygons.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <condition_variable>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>

namespace garage_road_model
{

visualization_msgs::msg::MarkerArray makeMarkers(
  const std::vector<RoadModelData> & roads,
  const std::string & frame_id,
  int64_t stamp_sec,
  uint32_t stamp_nanosec);

namespace
{

geometry_msgs::msg::Polygon toPolygon(const std::vector<Eigen::Vector2d> & points)
{
  geometry_msgs::msg::Polygon polygon;
  for (const auto & point : points) {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(point.x());
    p.y = static_cast<float>(point.y());
    polygon.points.push_back(p);
  }
  return polygon;
}

garage_road_model::msg::SideBoundary toSide(const SideBoundary & side)
{
  garage_road_model::msg::SideBoundary message;
  message.raw = toPolygon(side.raw);
  message.length = side.length;
  message.max_error = side.max_error;
  message.mean_error = side.mean_error;
  for (const auto & sample : side.samples) {
    garage_road_model::msg::RoadSample out;
    out.x = sample.x;
    out.y = sample.y;
    out.yaw = sample.yaw;
    out.s = sample.s;
    out.curvature = sample.curvature;
    out.corner = sample.corner;
    message.samples.push_back(out);
  }
  return message;
}

garage_road_model::msg::RoadModelArray toMessage(
  const std::vector<RoadModelData> & roads,
  uint64_t revision,
  const std::string & frame_id)
{
  garage_road_model::msg::RoadModelArray message;
  message.header.frame_id = frame_id;
  message.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  message.revision = revision;
  for (const auto & road : roads) {
    garage_road_model::msg::RoadModel item;
    item.source_index = road.source_index;
    item.boundary = toPolygon(road.boundary);
    item.start_cap = toPolygon(road.start_cap);
    item.end_cap = toPolygon(road.end_cap);
    item.side_a = toSide(road.side_a);
    item.side_b = toSide(road.side_b);
    item.valid = road.valid;
    item.stage = road.stage;
    item.reason = road.valid ? "" : road.stage + ": " + road.reason;
    message.roads.push_back(item);
  }
  return message;
}

std::string fingerprint(const std::vector<std::vector<Eigen::Vector2d>> & polygons)
{
  std::ostringstream text;
  text.setf(std::ios::fixed);
  text.precision(4);
  for (const auto & polygon : polygons) {
    text << '|';
    for (const auto & point : polygon) {
      text << point.x() << ',' << point.y() << ';';
    }
  }
  return text.str();
}

ModelConfig configFrom(rclcpp::Node & node)
{
  ModelConfig config;
  config.collinear_tolerance = node.get_parameter("collinear_tolerance").as_double();
  config.corner_angle_rad = node.get_parameter("corner_angle_rad").as_double();
  config.cap_side_ratio = node.get_parameter("cap_side_ratio").as_double();
  config.junction_angle_rad = node.get_parameter("junction_angle_rad").as_double();
  config.width_success_fraction = node.get_parameter("width_success_fraction").as_double();
  config.piece_angle_rad = node.get_parameter("piece_angle_rad").as_double();
  config.resample_spacing = node.get_parameter("resample_spacing").as_double();
  config.sample_spacing = node.get_parameter("sample_spacing").as_double();
  config.max_fit_error = node.get_parameter("max_fit_error").as_double();
  config.fit_attempts = static_cast<int>(node.get_parameter("fit_attempts").as_int());
  config.initial_lambda = node.get_parameter("initial_lambda").as_double();
  config.max_controls = static_cast<int>(node.get_parameter("max_controls").as_int());
  config.control_spacing = node.get_parameter("control_spacing").as_double();
  config.curve_check_spacing = node.get_parameter("curve_check_spacing").as_double();
  config.frame_id = node.get_parameter("frame_id").as_string();
  return config;
}

}  // namespace

class RoadModelNode : public rclcpp::Node
{
public:
  RoadModelNode()
  : Node("garage_road_model")
  {
    declare_parameter("collinear_tolerance", 0.05);
    declare_parameter("corner_angle_rad", 0.9);
    declare_parameter("cap_side_ratio", 0.75);
    declare_parameter("junction_angle_rad", 0.7);
    declare_parameter("width_success_fraction", 0.45);
    declare_parameter("piece_angle_rad", 0.5);
    declare_parameter("resample_spacing", 0.2);
    declare_parameter("sample_spacing", 0.2);
    declare_parameter("max_fit_error", 0.08);
    declare_parameter("fit_attempts", 5);
    declare_parameter("initial_lambda", 1.0);
    declare_parameter("max_controls", 48);
    declare_parameter("control_spacing", 1.2);
    declare_parameter("curve_check_spacing", 0.05);
    declare_parameter("frame_id", "map");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    model_pub_ = create_publisher<garage_road_model::msg::RoadModelArray>("/garage_road_models", qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/garage_road_model_markers", qos);
    polygon_sub_ = create_subscription<garage_utils_msgs::msg::Polygons>(
      "/garage_polygons", qos,
      [this](const garage_utils_msgs::msg::Polygons::SharedPtr msg) {onPolygons(msg);});
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        ModelConfig preview = configFrom(*this);
        for (const auto & parameter : parameters) {
          const std::string & name = parameter.get_name();
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            const double value = parameter.as_double();
            if (value <= 0.0 && name != "initial_lambda") {
              result.successful = false;
              result.reason = name + " 必须为正";
              return result;
            }
            if ((name == "cap_side_ratio" || name == "width_success_fraction") && value >= 1.0) {
              result.successful = false;
              result.reason = name + " 必须小于 1";
              return result;
            }
            if (name == "collinear_tolerance") {preview.collinear_tolerance = value;}
            else if (name == "corner_angle_rad") {preview.corner_angle_rad = value;}
            else if (name == "cap_side_ratio") {preview.cap_side_ratio = value;}
            else if (name == "junction_angle_rad") {preview.junction_angle_rad = value;}
            else if (name == "width_success_fraction") {preview.width_success_fraction = value;}
            else if (name == "piece_angle_rad") {preview.piece_angle_rad = value;}
            else if (name == "resample_spacing") {preview.resample_spacing = value;}
            else if (name == "sample_spacing") {preview.sample_spacing = value;}
            else if (name == "max_fit_error") {preview.max_fit_error = value;}
            else if (name == "initial_lambda") {preview.initial_lambda = value;}
            else if (name == "control_spacing") {preview.control_spacing = value;}
            else if (name == "curve_check_spacing") {preview.curve_check_spacing = value;}
          } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            const int value = static_cast<int>(parameter.as_int());
            if (value <= 0) {
              result.successful = false;
              result.reason = name + " 必须为正";
              return result;
            }
            if (name == "fit_attempts") {preview.fit_attempts = value;}
            else if (name == "max_controls") {preview.max_controls = value;}
          } else if (name == "frame_id") {
            preview.frame_id = parameter.as_string();
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pending_config_ = preview;
          parameters_dirty_ = true;
        }
        wake_.notify_one();
        return result;
      });
    worker_ = std::thread([this]() {loop();});
  }

  ~RoadModelNode() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void onPolygons(const garage_utils_msgs::msg::Polygons::SharedPtr & msg)
  {
    std::vector<std::vector<Eigen::Vector2d>> polygons;
    for (const auto & polygon : msg->polygons) {
      std::vector<Eigen::Vector2d> points;
      for (const auto & point : polygon.points) {
        points.emplace_back(point.x, point.y);
      }
      polygons.push_back(std::move(points));
    }
    const ModelConfig config = configFrom(*this);
    uint64_t seq = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      seq = ++latest_seq_;
      pending_ = polygons;
      pending_config_ = config;
      has_pending_ = true;
    }
    publishBuilding(polygons, seq);
    wake_.notify_one();
  }

  void publishBuilding(
    const std::vector<std::vector<Eigen::Vector2d>> & polygons,
    uint64_t revision)
  {
    std::vector<RoadModelData> roads;
    for (size_t i = 0; i < polygons.size(); ++i) {
      RoadModelData road;
      road.source_index = static_cast<uint32_t>(i);
      road.boundary = polygons[i];
      road.valid = false;
      road.stage = "建模";
      road.reason = "BUILDING";
      roads.push_back(road);
    }
    model_pub_->publish(toMessage(roads, revision, get_parameter("frame_id").as_string()));
  }

  void loop()
  {
    RoadModelBuilder builder;
    while (rclcpp::ok()) {
      std::vector<std::vector<Eigen::Vector2d>> polygons;
      ModelConfig config;
      uint64_t seq = 0;
      bool force = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this]() {return stop_ || has_pending_ || parameters_dirty_;});
        if (stop_) {
          return;
        }
        polygons = pending_;
        config = pending_config_;
        seq = latest_seq_;
        force = parameters_dirty_;
        has_pending_ = false;
        parameters_dirty_ = false;
      }
      const std::string key = fingerprint(polygons);
      if (!force && key == cached_key_ && have_cache_) {
        const auto message = toMessage(cached_roads_, seq, config.frame_id);
        model_pub_->publish(message);
        const auto stamp = message.header.stamp;
        marker_pub_->publish(makeMarkers(cached_roads_, config.frame_id, stamp.sec, stamp.nanosec));
        continue;
      }
      std::vector<RoadModelData> roads;
      if (!polygons.empty()) {
        for (size_t i = 0; i < polygons.size(); ++i) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
              return;
            }
          }
          roads.push_back(builder.build(polygons[i], static_cast<uint32_t>(i), config));
          RCLCPP_INFO(
            get_logger(), "道路 %zu %s %s 用时 %.1f ms",
            i, roads.back().valid ? "有效" : "失败",
            roads.back().valid ? "" : (roads.back().stage + " " + roads.back().reason).c_str(),
            roads.back().elapsed_ms);
        }
      }
      bool current = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current = resultIsCurrent(seq, latest_seq_, has_pending_);
        if (current) {
          cached_key_ = key;
          cached_roads_ = roads;
          have_cache_ = true;
        }
      }
      if (!current) {
        RCLCPP_INFO(get_logger(), "丢弃过期结果 revision %lu", static_cast<unsigned long>(seq));
        continue;
      }
      const auto message = toMessage(roads, seq, config.frame_id);
      model_pub_->publish(message);
      const auto stamp = message.header.stamp;
      marker_pub_->publish(makeMarkers(roads, config.frame_id, stamp.sec, stamp.nanosec));
    }
  }

  rclcpp::Publisher<garage_road_model::msg::RoadModelArray>::SharedPtr model_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<garage_utils_msgs::msg::Polygons>::SharedPtr polygon_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  std::vector<std::vector<Eigen::Vector2d>> pending_;
  ModelConfig pending_config_;
  uint64_t latest_seq_{0};
  bool has_pending_{false};
  bool parameters_dirty_{false};
  bool stop_{false};
  bool have_cache_{false};
  std::string cached_key_;
  std::vector<RoadModelData> cached_roads_;
};

}  // namespace garage_road_model

int runRoadModelNode(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<garage_road_model::RoadModelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
