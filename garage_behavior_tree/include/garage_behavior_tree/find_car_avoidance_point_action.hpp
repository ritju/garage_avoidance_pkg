

#ifndef GARAGE_BEHAVIOR_TREE__FIND_CAR_AVOIDANCE_POINT_ACTION_HPP_
#define GARAGE_BEHAVIOR_TREE__FIND_CAR_AVOIDANCE_POINT_ACTION_HPP_

#include <string>

#include "capella_ros_msg/action/find_car_avoidance_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "garage_behavior_tree/bt_action_node.hpp"

#include "garage_utils_msgs/msg/state.hpp"

namespace garage_utils_pkg
{

/**
 * @brief A garage_utils_pkg::BtActionNode class that wraps capella_ros_msg::action::FindCarAvoidancePoint
 */
class FindCarAvoidancePointAction : public BtActionNode<capella_ros_msg::action::FindCarAvoidancePoint>
{
public:
  /**
   * @brief A constructor for garage_utils_pkg::FindCarAvoidancePointAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  FindCarAvoidancePointAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::vector<geometry_msgs::msg::Polygon>>("polygons", "rects list"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("car_pose", "the pose of car"),
        BT::InputPort<geometry_msgs::msg::Vector3>("car_size", "the size of car"),
        BT::OutputPort<uint8_t>("state", "behavior tree state"),        
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("free_point", "the pose for robot parking."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "the pose for robot to navigating"),
        
      });
  }
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__FIND_CAR_AVOIDANCE_POINT_ACTION_HPP_
