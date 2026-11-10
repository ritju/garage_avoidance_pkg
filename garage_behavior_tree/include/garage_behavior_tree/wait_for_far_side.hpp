#ifndef GARAGE_BEHAVIOR_TREE__WAIT_FOR_FAR_SIDE_HPP_
#define GARAGE_BEHAVIOR_TREE__WAIT_FOR_FAR_SIDE_HPP_

#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "capella_ros_msg/msg/car_detect_array.hpp"

namespace garage_utils_pkg
{

// 远侧等待的看门节点，每个 tick 都要被执行：
//   SUCCESS  还可以继续等；连续无车满 clear_sec 时把 far_clear_since 写到黑板
//   FAILURE  连续有车满 busy_sec 或从 far_wait_start 起满 max_wait，放弃远侧
class WaitForFarSide : public BT::ActionNodeBase
{
public:
  WaitForFarSide(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  WaitForFarSide() = delete;

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("clear_sec", 3.0, "连续多少秒没有动态车可以放行"),
      BT::InputPort<double>("busy_sec", 4.0, "连续多少秒有动态车则改靠右"),
      BT::InputPort<double>("max_wait", 15.0, "从 far_wait_start 起的最长等待"),
      BT::InputPort<double>("stale_sec", 1.0, "超过这么久没有新消息则暂停 3 秒和 4 秒计时"),
      BT::InputPort<std::string>("topic_name", "/car_information_all", "动态车话题"),
    };
  }

private:
  void carCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg);
  bool exceededMaxWait(const rclcpp::Time & now);
  BT::NodeStatus giveUp();
  void resetState();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_sub_;

  std::mutex mutex_;
  double clear_sec_ = 3.0;
  double busy_sec_ = 4.0;
  double max_wait_ = 15.0;
  double stale_sec_ = 1.0;
  std::string topic_name_ = "/car_information_all";

  bool started_ = false;
  bool ever_received_ = false;
  bool last_has_car_ = false;
  rclcpp::Time last_msg_time_;
  rclcpp::Time last_tick_time_;
  rclcpp::Time last_log_time_;
  double clear_acc_ = 0.0;
  double busy_acc_ = 0.0;
  double clear_since_ = 0.0;
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__WAIT_FOR_FAR_SIDE_HPP_
