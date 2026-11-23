#include <string>

#include "garage_behavior_tree/far_side_guard.hpp"

namespace garage_utils_pkg
{

RunOnce::RunOnce(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::NodeStatus RunOnce::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    done_ = false;
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (done_) {
    return BT::NodeStatus::SUCCESS;
  }

  const BT::NodeStatus child_state = child_node_->executeTick();
  if (child_state == BT::NodeStatus::SUCCESS) {
    done_ = true;
  }
  return child_state;
}

void RunOnce::halt()
{
  done_ = false;
  BT::DecoratorNode::halt();
}

}  // namespace garage_utils_pkg

namespace garage_utils_pkg
{

FarSideGuard::FarSideGuard(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

bool FarSideGuard::blackboardBool(const std::string & key) const
{
  bool value = false;
  config().blackboard->get(key, value);
  return value;
}

uint8_t FarSideGuard::blackboardSide() const
{
  uint8_t side = 0;
  config().blackboard->get("point_side", side);
  return side;
}

BT::NodeStatus FarSideGuard::tick()
{
  std::string command;
  if (!getInput("command", command)) {
    RCLCPP_ERROR(node_->get_logger(), "FarSideGuard 缺少 command");
    return BT::NodeStatus::FAILURE;
  }

  auto bb = config().blackboard;

  if (command == "refresh") {
    double suppress_until = 0.0;
    bb->get("suppress_until", suppress_until);
    if (suppress_until > 0.0 && node_->now().seconds() >= suppress_until) {
      bb->set("allow_far", true);
      bb->set("suppress_until", 0.0);
      RCLCPP_INFO(node_->get_logger(), "远侧屏蔽结束，恢复远侧找点");
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (command == "has_approved") {
    return blackboardBool("approved_ready") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if (command == "has_far_pending") {
    if (blackboardBool("approved_ready")) {
      return BT::NodeStatus::FAILURE;
    }
    return blackboardBool("far_pending") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if (command == "is_near_or_external") {
    const uint8_t side = blackboardSide();
    return (side == 1 || side == 3) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if (command == "is_far") {
    return blackboardSide() == 2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if (command == "confirm_far") {
    double clear_since = 0.0;
    double find_sent = 0.0;
    bb->get("far_clear_since", clear_since);
    bb->get("find_goal_sent", find_sent);
    if (clear_since > 0.0 && find_sent >= clear_since) {
      RCLCPP_INFO(node_->get_logger(), "远侧点确认通过：找点发出时已连续无车");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (command == "approve") {
    geometry_msgs::msg::PoseStamped pose;
    if (!bb->get("saved_candidate", pose)) {
      RCLCPP_WARN(node_->get_logger(), "没有可批准的候选点");
      return BT::NodeStatus::FAILURE;
    }
    bb->set("approved_point", pose);
    bb->set("approved_ready", true);
    bb->set("far_pending", false);
    bb->set("far_wait_start", 0.0);
    bb->set("far_give_up", false);
    bb->set("right_edge_active", false);
    RCLCPP_INFO(node_->get_logger(),
      "批准停靠点 (%.2f, %.2f) side=%u",
      pose.pose.position.x, pose.pose.position.y, blackboardSide());
    return BT::NodeStatus::SUCCESS;
  }

  if (command == "commit") {
    geometry_msgs::msg::PoseStamped empty;
    bb->set("far_pending", false);
    bb->set("far_wait_start", 0.0);
    bb->set("far_give_up", false);
    bb->set("approved_ready", false);
    bb->set("allow_far", false);
    bb->set("suppress_until", 0.0);
    bb->set("tracking_armed", false);
    bb->set("far_clear_since", 0.0);
    bb->set("right_edge_active", true);
    bb->set("saved_candidate", empty);
    bb->set("candidate_point", empty);
    bb->set("point_side", static_cast<uint8_t>(0));
    RCLCPP_INFO(node_->get_logger(), "进入靠右路径，先屏蔽远侧找点");
    return BT::NodeStatus::SUCCESS;
  }

  if (command == "arm_tracking") {
    // 同一次靠右里的重规划、恢复重试都不重新起算。
    if (blackboardBool("tracking_armed")) {
      return BT::NodeStatus::SUCCESS;
    }
    bb->set("tracking_armed", true);
    double suppress_sec = 5.0;
    getInput("suppress_sec", suppress_sec);
    bb->set("suppress_until", node_->now().seconds() + suppress_sec);
    bb->set("allow_far", false);
    RCLCPP_INFO(node_->get_logger(), "靠右跟踪开始，%.1f 秒内不采纳远侧点", suppress_sec);
    return BT::NodeStatus::SUCCESS;
  }

  if (command == "yield") {
    return BT::NodeStatus::RUNNING;
  }

  if (command == "give_up_or_wait") {
    return blackboardBool("far_give_up") ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
  }

  if (command == "hold_if_point") {
    if (blackboardBool("approved_ready") || blackboardBool("far_pending")) {
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (command == "skip_initial") {
    const bool skip = blackboardBool("far_pending") || blackboardBool("approved_ready") ||
      blackboardBool("right_edge_active");
    return skip ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(node_->get_logger(), "未知 FarSideGuard command: %s", command.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace garage_utils_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<garage_utils_pkg::FarSideGuard>("FarSideGuard");
  factory.registerNodeType<garage_utils_pkg::RunOnce>("RunOnce");
}
