#ifndef GARAGE_BEHAVIOR_TREE__STOP_WHEN_CARS_GONE_HPP_
#define GARAGE_BEHAVIOR_TREE__STOP_WHEN_CARS_GONE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "capella_ros_msg/msg/car_detect_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace garage_utils_pkg
{

// 只包住靠右阶段；每次 tick 都检查动车超时，包括子节点等待 Action 结果期间。
class StopWhenCarsGone : public BT::DecoratorNode
{
public:
  StopWhenCarsGone(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("idle_timeout", 3.0, "连续未收到非空动车消息的退出时间（秒）"),
      BT::InputPort<std::string>("topic_name", "/car_information", "动车信息话题")
    };
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr subscription_;
  rclcpp::Time last_car_time_;
  double idle_timeout_{3.0};
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__STOP_WHEN_CARS_GONE_HPP_
