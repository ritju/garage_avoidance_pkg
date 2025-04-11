
#ifndef GARAGE_BEHAVIOR_TREE__WELT_MODEL_NODE_HPP_
#define GARAGE_BEHAVIOR_TREE__WELT_MODEL_NODE_HPP_

#include <string>

#include "garage_behavior_tree/bt_action_node.hpp"
#include "capella_ros_msg/action/welt.hpp"

namespace garage_utils_pkg
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps capella_ros_msg::action::RelocalizationSpin
 */
class WeltModelNode : public BtActionNode<capella_ros_msg::action::Welt>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  WeltModelNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
  
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "welt to goal pose"),
      });
  }

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__RELOCALIZATION_SPIN_ACTION_HPP_
