

#ifndef GARAGE_NAVIGATOR__NAVIGATOR_HPP_
#define GARAGE_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"

namespace garage_utils_pkg
{
/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template<class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<garage_utils_pkg::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator()
  {
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() = default;

  /**
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @return bool If successful
   */
  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names, std::shared_ptr<tf2_ros::Buffer> tf)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Navigator::onLoop, this),
      std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
      std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf); 

    return configure(parent_node) && ok;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_activate()
  {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate()
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup()
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    bool goal_accepted = goalReceived(goal);

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
  {
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  /**
   * @brief Method to activate any threads involved in execution.
   */
  virtual bool activate() {return true;}

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() {return true;}

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Garage_Navigator")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_NAVIGATOR__NAVIGATOR_HPP_
