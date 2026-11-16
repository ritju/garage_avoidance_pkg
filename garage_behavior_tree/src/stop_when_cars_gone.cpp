#include "garage_behavior_tree/stop_when_cars_gone.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace garage_utils_pkg
{

StopWhenCarsGone::StopWhenCarsGone(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  std::string topic_name;
  if (!getInput("topic_name", topic_name) || topic_name.empty()) {
    throw BT::RuntimeError("StopWhenCarsGone: topic_name 不能为空");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>(
    topic_name, rclcpp::QoS(1).reliable().durability_volatile(),
    [this](capella_ros_msg::msg::CarDetectArray::ConstSharedPtr msg) {
      if (!msg->results.empty()) {
        last_car_time_ = node_->now();
      }
    }, options);
}

BT::NodeStatus StopWhenCarsGone::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    if (!getInput("idle_timeout", idle_timeout_) ||
      !std::isfinite(idle_timeout_) || idle_timeout_ <= 0.0)
    {
      throw BT::RuntimeError("StopWhenCarsGone: idle_timeout 必须是大于 0 的有限秒数");
    }
    last_car_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "靠右动车监测开始，连续 %.1f 秒无动车则结束", idle_timeout_);
  }
  setStatus(BT::NodeStatus::RUNNING);
  callback_executor_.spin_some();

  const auto now = node_->now();
  // 仿真时钟回退时重新计时，暂停时不累加无车时间。
  if (now < last_car_time_) {
    last_car_time_ = now;
  }
  if ((now - last_car_time_).seconds() >= idle_timeout_) {
    // halt 会向运行中的找点、规划、跟踪和恢复 Action 发送取消。
    // 返回 SUCCESS，避免 Fallback 把“车已离开”当成找点失败而进入停靠分支。
    haltChild();
    RCLCPP_INFO(node_->get_logger(),
      "靠右期间连续 %.1f 秒未收到动车，已中断靠右和找点，本次避车成功结束", idle_timeout_);
    return BT::NodeStatus::SUCCESS;
  }

  const auto child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    resetChild();
  }
  return child_status;
}

void StopWhenCarsGone::halt()
{
  BT::DecoratorNode::halt();
}

}  // namespace garage_utils_pkg

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<garage_utils_pkg::StopWhenCarsGone>("StopWhenCarsGone");
}
