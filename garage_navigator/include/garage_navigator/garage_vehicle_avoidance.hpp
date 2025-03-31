#ifndef GARAGE_NAVIGATOR__GARAGE_VEHICLE_AVOIDANCE_HPP_
#define GARAGE_NAVIGATOR__GARAGE_VEHICLE_AVOIDANCE_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "garage_navigator/navigator.hpp"

#include "garage_utils_msgs/action/garage_vehicle_avoidance.hpp"
#include "garage_utils_msgs/msg/state.hpp"

namespace garage_utils_pkg
{

/**
 * @class GarageVehicleAvoidanceNavigator
 * @brief A navigator for vehicle avoidance in garage
 */
class GarageVehicleAvoidanceNavigator
  : public garage_utils_pkg::Navigator<garage_utils_msgs::action::GarageVehicleAvoidance>
{
public:
  using ActionT = garage_utils_msgs::action::GarageVehicleAvoidance;

  /**
   * @brief A constructor for GarageVehicleAvoidanceNavigator
   */
  GarageVehicleAvoidanceNavigator()
  : Navigator() {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   * @param pose Pose received via atopic
   */
  void onGoalPoseReceived(const std::vector<geometry_msgs::msg::Polygon> polygons, geometry_msgs::msg::PoseStamped car_pose);

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() {return std::string("garage_vehicle_avoidance");}

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * referenced while populating the result.
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Goal pose initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  std::string polygons_blackboard_id_;
  std::string car_pose_blackboard_id_;
  std::string path_blackboard_id_;
  std::string car_size_id_;
  std::string state_id_;
  std::string goals_blackboard_id_;

};

}  // namespace garage_utils_pkg

#endif  // #define GARAGE_NAVIGATOR__GARAGE_VEHICLE_AVOIDANCE_HPP_

