#ifndef GARAGE_BEHAVIOR_TREE__IS_CAR_MOVING_
#define GARAGE_BEHAVIOR_TREE__IS_CAR_MOVING_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "capella_ros_msg/msg/car_detect_array.hpp"

namespace garage_utils_pkg
{

/**
 * @brief A BT::ConditionNode that listens to localizaiton_score topic and
 * returns SUCCESS when localizaiton_score is high and FAILURE otherwise
 */
class IsCarMoving : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsCarMoving(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsCarMoving() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  /**
   * @brief Callback function for localization_score topic
   * @param msg Shared pointer to std_msgs::msg::Float32 message
   */
  void iscollisionCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_info_sub;
  std::string is_collision_topic_;
  bool is_moving_ = false;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_LOCALIZATION_STATUS_GOOD_
