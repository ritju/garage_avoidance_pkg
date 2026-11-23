#include <algorithm>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "garage_behavior_tree/wait_for_far_side.hpp"

namespace garage_utils_pkg
{

WaitForFarSide::WaitForFarSide(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("clear_sec", clear_sec_);
  getInput("busy_sec", busy_sec_);
  getInput("max_wait", max_wait_);
  getInput("stale_sec", stale_sec_);
  getInput("topic_name", topic_name_);

  RCLCPP_INFO(node_->get_logger(),
    "WaitForFarSide: topic=%s clear=%.1f busy=%.1f max_wait=%.1f stale=%.1f",
    topic_name_.c_str(), clear_sec_, busy_sec_, max_wait_, stale_sec_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  car_sub_ = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>(
    topic_name_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&WaitForFarSide::carCallback, this, std::placeholders::_1),
    sub_option);
  last_msg_time_ = node_->now();
  last_tick_time_ = node_->now();
  last_log_time_ = node_->now();
}

bool WaitForFarSide::exceededMaxWait(const rclcpp::Time & now)
{
  double far_wait_start = 0.0;
  if (!config().blackboard->get("far_wait_start", far_wait_start) || far_wait_start <= 0.0) {
    return false;
  }
  return (now.seconds() - far_wait_start) >= max_wait_;
}

BT::NodeStatus WaitForFarSide::giveUp()
{
  RCLCPP_WARN(node_->get_logger(), "放弃远侧，改走靠右");
  config().blackboard->set("far_give_up", true);
  resetState();
  return BT::NodeStatus::FAILURE;
}

void WaitForFarSide::resetState()
{
  started_ = false;
  clear_acc_ = 0.0;
  busy_acc_ = 0.0;
}

BT::NodeStatus WaitForFarSide::tick()
{
  callback_group_executor_.spin_some();

  const rclcpp::Time now = node_->now();
  bool has_car = false;
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_car = last_has_car_;
    fresh = ever_received_ && (now - last_msg_time_).seconds() <= stale_sec_;
  }

  if (!started_) {
    started_ = true;
    last_tick_time_ = now;
    clear_acc_ = 0.0;
    busy_acc_ = 0.0;
    RCLCPP_INFO(node_->get_logger(), "开始远侧等待");
  }

  const double dt = std::max(0.0, (now - last_tick_time_).seconds());
  last_tick_time_ = now;

  // 断流时 3 秒和 4 秒暂停，15 秒墙钟继续走。
  if (fresh) {
    if (has_car) {
      clear_acc_ = 0.0;
      busy_acc_ += dt;
    } else {
      busy_acc_ = 0.0;
      clear_acc_ += dt;
    }
  }

  if ((now - last_log_time_).seconds() >= 1.0) {
    last_log_time_ = now;
    double far_wait_start = 0.0;
    config().blackboard->get("far_wait_start", far_wait_start);
    RCLCPP_INFO(node_->get_logger(),
      "远侧等待：有车=%d 数据新鲜=%d 无车累计=%.1f 有车累计=%.1f 已等=%.1f",
      has_car, fresh, clear_acc_, busy_acc_,
      far_wait_start > 0.0 ? now.seconds() - far_wait_start : 0.0);
  }

  if (fresh && busy_acc_ >= busy_sec_) {
    RCLCPP_WARN(node_->get_logger(), "连续 %.1f 秒检测到动态车", busy_sec_);
    return giveUp();
  }
  if (exceededMaxWait(now)) {
    RCLCPP_WARN(node_->get_logger(), "远侧等待达到 %.1f 秒", max_wait_);
    return giveUp();
  }
  if (fresh && !has_car && clear_acc_ >= clear_sec_) {
    RCLCPP_INFO(node_->get_logger(),
      "连续 %.1f 秒没有动态车，直接使用已保存的远侧点", clear_sec_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForFarSide::halt()
{
  resetState();
  setStatus(BT::NodeStatus::IDLE);
}

void WaitForFarSide::carCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ever_received_ = true;
  last_has_car_ = !msg->results.empty();
  last_msg_time_ = node_->now();
}

}  // namespace garage_utils_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<garage_utils_pkg::WaitForFarSide>("WaitForFarSide");
}
