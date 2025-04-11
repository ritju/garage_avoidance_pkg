#include <string>

#include "garage_behavior_tree/is_car_moving.hpp"

namespace garage_utils_pkg
{

IsCarMoving::IsCarMoving(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_collision_topic_("/car_information"),
  is_moving_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  car_info_sub = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>(
    is_collision_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&IsCarMoving::iscollisionCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsCarMoving::tick()
{
  callback_group_executor_.spin_some();
  if (is_moving_) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving!");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void IsCarMoving::iscollisionCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
{
  if(msg->results.size()==0){
    is_moving_ = false;
    return;
  } 
  for(int i=0;i<msg->results.size();i++){
    if(msg->results[i].speed.twist.linear.x != 0 || (msg->results[i].speed.twist.angular.z != 0)){
      is_moving_ = true;
      break;
    }
  } 
  is_moving_ = false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsCarMoving>("IsCarMoving");
}
