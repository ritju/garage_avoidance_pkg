#ifndef GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_
#define GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "capella_ros_msg/msg/car_detect_array.hpp"

namespace garage_utils_pkg
{

class WaitForCarsGone : public BT::ActionNode
{
public:

  WaitForCarsGone(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  WaitForCarsGone() = delete;

  BT::NodeStatus tick() override;


  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("idle_timeout", 15.0,
        "连续多少秒没有检测到车则结束避车等待"),
      BT::InputPort<double>("max_wait", -1.0,
        "最大等待总时长（秒），<=0 表示不限"),
      BT::InputPort<std::string>("topic_name", "/car_information",
        "车辆信息话题"),
    };
  }

private:

  void carInformationCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_info_sub_;

  std::mutex mutex_;        
  double idle_timeout_ = 15.0;
  double max_wait_ = -1.0;
  std::string topic_name_ = "/car_information";

  bool started_ = false;
  rclcpp::Time start_time_;    // 开始等待的时刻
  rclcpp::Time last_car_time_; // 最近一次收到车的时间
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_
