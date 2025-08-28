
#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "garage_navigator/garage_vehicle_avoidance.hpp"

namespace garage_utils_pkg
{

bool
GarageVehicleAvoidanceNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("polygons_blackboard_id")) {
    node->declare_parameter("polygons_blackboard_id", std::string("polygons"));
  }
  polygons_blackboard_id_ = node->get_parameter("polygons_blackboard_id").as_string();

  if (!node->has_parameter("car_pose_blackboard_id")) {
    node->declare_parameter("car_pose_blackboard_id", std::string("car_pose"));
  }
  car_pose_blackboard_id_ = node->get_parameter("car_pose_blackboard_id").as_string();

  if (!node->has_parameter("car_size_blackboard_id"))
  {
    node->declare_parameter("car_size_blackboard_id", std::string("car_size"));
  }
  car_size_blackboard_id_ = node->get_parameter("car_size_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("garage_path"));
  }
  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  if (!node->has_parameter("goals_blackboard_id")) {
    node->declare_parameter("goals_blackboard_id", std::string("poses"));
  }
  goals_blackboard_id_ = node->get_parameter("goals_blackboard_id").as_string();

  if (!node->has_parameter("state_id")) {
    node->declare_parameter("state_id", std::string("state"));
  }
  state_id_ = node->get_parameter("state_id").as_string();

  if (!node->has_parameter("nav2pose_goal_blackboard_id")) {
    node->declare_parameter("nav2pose_goal_blackboard_id", std::string("goal"));
  }
  nav2pose_goal_blackboard_id_ = node->get_parameter("nav2pose_goal_blackboard_id").as_string();

  if (!node->has_parameter("nav2pose_path_blackboard_id")) {
    node->declare_parameter("nav2pose_path_blackboard_id", std::string("path"));
  }
  nav2pose_path_blackboard_id_ = node->get_parameter("nav2pose_path_blackboard_id").as_string();

  if (!node->has_parameter("is_car_moving_car_distance_thr_id")) {
    node->declare_parameter("is_car_moving_car_distance_thr_id", std::string("is_car_moving_car_distance_thr_id"));
  }
  is_car_moving_car_distance_thr_id_ = node->get_parameter("is_car_moving_car_distance_thr_id").as_string();

  if (!node->has_parameter("is_car_moving_time_predict_id")) {
    node->declare_parameter("is_car_moving_time_predict_id", std::string("is_car_moving_time_predict_id"));
  }
  is_car_moving_time_predict_id_ = node->get_parameter("is_car_moving_time_predict_id").as_string();

  auto tmp_action_client_find_ = rclcpp_action::create_client<capella_ros_msg::action::FindCarAvoidancePoint>(node, "find_car_avoidance_point_action");
  auto tmp_action_client_compute_ = rclcpp_action::create_client<garage_utils_msgs::action::ComputeRightEdgePath>(node, "compute_right_edge_path_action_server");
  auto tmp_action_client_welt_ = rclcpp_action::create_client<capella_ros_msg::action::Welt>(node, "welt_model_node");
  auto tmp_action_client_wait_ = rclcpp_action::create_client<nav2_msgs::action::Wait>(node, "wait");

  if (!tmp_action_client_find_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action /find_car_avoidance_point_action is not on line after 10 seconds.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Action /find_car_avoidance_point_action is on line.");

  if (!tmp_action_client_compute_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action /compute_right_edge_path_action_server is not on line after 10 seconds.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Action /compute_right_edge_path_action_server is on line.");

  if (!tmp_action_client_welt_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action /welt_model_node is not on line after 10 seconds.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Action  /welt_model_node is on line.");

  if (!tmp_action_client_wait_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action /wait is not on line after 10 seconds.");
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "Action /wait is on line.");
  
  return true;
}

std::string
GarageVehicleAvoidanceNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_garage_avoidance_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("garage_navigator");
    node->declare_parameter<std::string>(
      "default_nav_garage_avoidance_bt_xml",
      pkg_share_dir +
      "/behavior_trees/garage.xml");
  }

  node->get_parameter("default_nav_garage_avoidance_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
GarageVehicleAvoidanceNavigator::cleanup()
{
  return true;
}

bool
GarageVehicleAvoidanceNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  if (goal->polygons.size() == 0)
  {
    RCLCPP_ERROR(logger_, "goal->polygons.size() == 0");
    return false;
  }

  if (goal->cars_information.results.size() == 0)
  {
    RCLCPP_ERROR(logger_, "goal->cars_information.results.size() == 0");
    return false;
  }
  

  initializeGoalPose(goal);

  return true;
}

void
GarageVehicleAvoidanceNavigator::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  RCLCPP_INFO(logger_, "goalCompleted callback");
  switch (final_bt_status)
  {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      result->success = true;
      result->reason = "goal completed.";
      break;
    case nav2_behavior_tree::BtStatus::FAILED:
      result->success = false;
      result->reason = "failed";
      break;
    case nav2_behavior_tree::BtStatus::CANCELED:
      result->success = false;
      result->reason = "canceled";
      break;
  }
}

void
GarageVehicleAvoidanceNavigator::onLoop()
{
  // action server feedback 
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  uint8_t state_number = 88;  
  blackboard->get<uint8_t>(state_id_, state_number);
  // RCLCPP_INFO(logger_, "get state: %d", state_number);
  feedback_msg->state = state_number;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
GarageVehicleAvoidanceNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
GarageVehicleAvoidanceNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    logger_, "Begin to execute garage_vehicle_avoidance program.");

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();

  // Update the polygons/car_pose/car_size/state on the blackboard
  blackboard->set<std::vector<geometry_msgs::msg::Polygon>>(polygons_blackboard_id_, goal->polygons);
  blackboard->set<geometry_msgs::msg::PoseStamped>(car_pose_blackboard_id_, goal->cars_information.results[0].pose);
  blackboard->set<geometry_msgs::msg::Vector3>(car_size_blackboard_id_, goal->cars_information.results[0].size);
  // RCLCPP_INFO(logger_, "initializeGoalPose set state: %d", garage_utils_msgs::msg::State::INIT);
  blackboard->set<uint8_t>(state_id_, garage_utils_msgs::msg::State::INIT);
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
  blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>(goals_blackboard_id_, std::vector<geometry_msgs::msg::PoseStamped>());
  blackboard->set<geometry_msgs::msg::PoseStamped>(nav2pose_goal_blackboard_id_, geometry_msgs::msg::PoseStamped());
  blackboard->set<nav_msgs::msg::Path>(nav2pose_path_blackboard_id_, nav_msgs::msg::Path());
  blackboard->set<double>(is_car_moving_car_distance_thr_id_, 4.0);
  blackboard->set<double>(is_car_moving_time_predict_id_, 3.0);
}

}  // namespace garage_utils_pkg
