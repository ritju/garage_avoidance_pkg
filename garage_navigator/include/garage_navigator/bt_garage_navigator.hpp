// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GARAGE_NAVIGATOR__BT_GARAGE_NAVIGATOR_HPP_
#define GARAGE_NAVIGATOR__BT_GARAGE_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "garage_navigator/garage_vehicle_avoidance.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace garage_utils_pkg
{

/**
 * @class garage_utils_pkg::BtNavigator
 * @brief 
 */
class BtNavigator : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for garage_utils_pkg::BtNavigator class
   * @param options Additional options to control creation of the node.
   */
  explicit BtNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for garage_utils_pkg::BtNavigator class
   */
  ~BtNavigator();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "NavigationToPose"; subscription to
   * "goal_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  std::unique_ptr<garage_utils_pkg::Navigator<garage_utils_msgs::action::GarageVehicleAvoidance>> garage_navigator_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

}  // namespace garage_utils_pkg

#endif  // GARAGE_NAVIGATOR__BT_GARAGE_NAVIGATOR_HPP_
