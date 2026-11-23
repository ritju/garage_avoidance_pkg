#ifndef GARAGE_BEHAVIOR_TREE__FAR_SIDE_GUARD_HPP_
#define GARAGE_BEHAVIOR_TREE__FAR_SIDE_GUARD_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace garage_utils_pkg
{

class FarSideGuard : public BT::ActionNodeBase
{
public:
  FarSideGuard(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  FarSideGuard() = delete;

  BT::NodeStatus tick() override;
  void halt() override {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command", "approve、commit、arm_tracking、refresh 等"),
      BT::InputPort<double>("suppress_sec", 5.0, "靠右跟踪开始后屏蔽远侧的秒数"),
    };
  }

private:
  bool blackboardBool(const std::string & key) const;
  uint8_t blackboardSide() const;

  rclcpp::Node::SharedPtr node_;
};

// 子节点成功一次之后一直返回 SUCCESS。
// Nav2 的 SingleTrigger 成功后再次 tick 会返回 FAILURE，放在 PipelineSequence 里会把正在跟踪的路径停掉。
class RunOnce : public BT::DecoratorNode
{
public:
  RunOnce(const std::string & name, const BT::NodeConfiguration & conf);

  RunOnce() = delete;

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts() {return {};}

private:
  bool done_{false};
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__FAR_SIDE_GUARD_HPP_
