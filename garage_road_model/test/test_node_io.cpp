#include <garage_road_model/msg/road_model_array.hpp>
#include <garage_utils_msgs/msg/polygons.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace
{

class NodeProcess
{
public:
  explicit NodeProcess(const std::string & path)
  {
    pid_ = fork();
    if (pid_ == 0) {
      execl(path.c_str(), path.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    if (pid_ < 0) {
      throw std::runtime_error("无法启动道路模型节点");
    }
  }

  ~NodeProcess()
  {
    if (pid_ > 0) {
      kill(pid_, SIGINT);
      waitpid(pid_, nullptr, 0);
    }
  }

private:
  pid_t pid_{-1};
};

garage_utils_msgs::msg::Polygons rectangle()
{
  garage_utils_msgs::msg::Polygons message;
  geometry_msgs::msg::Polygon polygon;
  for (const auto & point : {std::pair<float, float>{0.F, 0.F}, {20.F, 0.F}, {20.F, 2.F}, {0.F, 2.F}}) {
    geometry_msgs::msg::Point32 corner;
    corner.x = point.first;
    corner.y = point.second;
    polygon.points.push_back(corner);
  }
  message.polygons.push_back(polygon);
  return message;
}

garage_utils_msgs::msg::Polygons bowtie()
{
  garage_utils_msgs::msg::Polygons message;
  geometry_msgs::msg::Polygon polygon;
  for (const auto & point : {std::pair<float, float>{0.F, 0.F}, {2.F, 2.F}, {0.F, 2.F}, {2.F, 0.F}}) {
    geometry_msgs::msg::Point32 corner;
    corner.x = point.first;
    corner.y = point.second;
    polygon.points.push_back(corner);
  }
  message.polygons.push_back(polygon);
  return message;
}

}  // namespace

TEST(RoadModelNode, PublishesFitAndClearsEmptyInput)
{
  const char * path = std::getenv("ROAD_MODEL_NODE");
  ASSERT_NE(path, nullptr);
  NodeProcess process(path);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("road_model_probe");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  garage_road_model::msg::RoadModelArray latest;
  bool got = false;
  auto sub = node->create_subscription<garage_road_model::msg::RoadModelArray>(
    "/garage_road_models", qos,
    [&](const garage_road_model::msg::RoadModelArray::SharedPtr msg) {
      latest = *msg;
      got = true;
    });
  auto pub = node->create_publisher<garage_utils_msgs::msg::Polygons>("/garage_polygons", qos);
  const auto wait_for = [&](const auto & ready) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
      while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        if (ready()) {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      return false;
    };
  pub->publish(rectangle());
  ASSERT_TRUE(wait_for([&]() {
      return got && !latest.roads.empty() && latest.roads.front().valid;
    }));
  ASSERT_EQ(latest.roads.size(), 1U);
  EXPECT_TRUE(latest.roads.front().valid) << latest.roads.front().reason;
  EXPECT_EQ(latest.roads.front().source_index, 0U);
  EXPECT_GT(latest.roads.front().side_a.length, 18.0);
  EXPECT_GT(latest.roads.front().side_b.length, 18.0);
  const uint64_t fitted_revision = latest.revision;

  got = false;
  pub->publish(garage_utils_msgs::msg::Polygons());
  ASSERT_TRUE(wait_for([&]() {
      return got && latest.revision != fitted_revision && latest.roads.empty();
    }));
  EXPECT_TRUE(latest.roads.empty());

  got = false;
  pub->publish(bowtie());
  const uint64_t empty_revision = latest.revision;
  ASSERT_TRUE(wait_for([&]() {
      return got && latest.revision != empty_revision && !latest.roads.empty() &&
             !latest.roads.front().valid &&
             latest.roads.front().reason.find("自交") != std::string::npos;
    }));
  ASSERT_FALSE(latest.roads.empty());
  EXPECT_FALSE(latest.roads.front().valid);
  EXPECT_NE(latest.roads.front().reason.find("自交"), std::string::npos);
  EXPECT_EQ(latest.roads.front().source_index, 0U);
  rclcpp::shutdown();
}
