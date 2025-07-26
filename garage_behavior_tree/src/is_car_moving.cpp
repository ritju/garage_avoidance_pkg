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
  car_distance_thr_ = config().blackboard->get<double>("car_distance_thr");  
  time_predict_ = config().blackboard->get<double>("time_predict");
  tf_buffer_ = config().blackboard->get<tf2_ros::Buffer::SharedPtr>("tf_buffer");
  RCLCPP_INFO(node_->get_logger(), "car_distance_thr: %f", car_distance_thr_);
  RCLCPP_INFO(node_->get_logger(), "time_predict: %f", time_predict_);

  // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  car_info_sub = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>(
    is_collision_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&IsCarMoving::carInformationCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsCarMoving::tick()
{
  callback_group_executor_.spin_some();
  if (is_moving_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger() , *node_->get_clock(), 1000, "机器人附近有车!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsCarMoving::carInformationCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
{
  if(msg->results.size()==0){
    is_moving_ = false;
    return;
  }

  if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, "map", "base_link", 0.1))
  {
    RCLCPP_WARN(node_->get_logger(), "无法获取机器人的位姿");
    is_moving_ = true;
    return;
  }

  for(size_t i = 0; i < msg->results.size(); i++)
  {
    auto car_pose = msg->results[i].pose;
    auto car_twist = msg->results[i].speed;

    robot_x = robot_pose.pose.position.x;
    robot_y = robot_pose.pose.position.y;
    robot_theta = tf2::getYaw(robot_pose.pose.orientation);
    car_x = car_pose.pose.position.x;
    car_y = car_pose.pose.position.y;

    double distance = std::hypot(robot_x - car_x, robot_y - car_y);
    if(distance <= car_distance_thr_)
    {
      is_moving_ = true;
      break;
    }
    else
    {
      if (distance - car_twist.twist.linear.x * time_predict_ < car_distance_thr_)
      {
        is_moving_ = true;
        break;
      }
      else
      {
        continue;
      }
    }
  } 
  is_moving_ = false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<garage_utils_pkg::IsCarMoving>("IsCarMoving");
}
