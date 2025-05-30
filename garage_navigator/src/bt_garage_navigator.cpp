
#include "garage_navigator/bt_garage_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "garage_behavior_tree/bt_conversions.hpp"

namespace garage_utils_pkg
{

BtNavigator::BtNavigator(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("bt_garage_navigator", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    // "compute_right_edge_path_action",
    "nav2_pipeline_sequence_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_wait_action_bt_node"
  };

  declare_parameter("plugin_lib_names", plugin_libs);
}

BtNavigator::~BtNavigator()
{
}

nav2_util::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  garage_navigator_ = std::make_unique<garage_utils_pkg::GarageVehicleAvoidanceNavigator>();


  if (!garage_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, tf_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }


  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!garage_navigator_->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();
  // RCLCPP_INFO(get_logger(), "after bound");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!garage_navigator_->on_deactivate() ) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");


  if (!garage_navigator_->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  garage_navigator_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace garage_utils_pkg

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(garage_utils_pkg::BtNavigator)
