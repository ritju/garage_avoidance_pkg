#include <string>

#include "garage_behavior_tree/wait_for_cars_gone.hpp"

namespace garage_utils_pkg
{

WaitForCarsGone::WaitForCarsGone(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("idle_timeout", idle_timeout_);
  getInput("max_wait", max_wait_);
  getInput("topic_name", topic_name_);

  RCLCPP_INFO(node_->get_logger(),
    "WaitForCarsGone: topic=%s, idle_timeout=%.1f, max_wait=%.1f",
    topic_name_.c_str(), idle_timeout_, max_wait_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  car_info_sub_ = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&WaitForCarsGone::carInformationCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus WaitForCarsGone::tick()
{
  callback_group_executor_.spin_some();

  rclcpp::Time now = node_->now();

  if (!started_) {
    started_ = true;
    start_time_ = now;
    last_car_time_ = now;
  }

  // 连续 idle_timeout 秒没有收到车 → 避车等待结束
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((now - last_car_time_).seconds() >= idle_timeout_) {
      RCLCPP_INFO(node_->get_logger(),
        "连续 %.1f 秒没有检测到车，避车等待结束", idle_timeout_);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 最大等待总时长
  if (max_wait_ > 0.0 && (now - start_time_).seconds() >= max_wait_) {
    RCLCPP_WARN(node_->get_logger(),
      "避车等待超过最大时长 %.1f 秒，强制结束", max_wait_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForCarsGone::halt()
{
  started_ = false;
  BT::ActionNode::halt();
}

void WaitForCarsGone::carInformationCallback(
  capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
{
  if (msg->results.empty()) {
    return;  // 没有检测到车，不重置计时
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_car_time_ = node_->now();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "避车等待期间仍检测到 %zu 辆车，重置计时", msg->results.size());
}

}  // namespace garage_utils_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<garage_utils_pkg::WaitForCarsGone>("WaitForCarsGone");
}
