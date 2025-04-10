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

#include <memory>
#include <string>

#include "garage_behavior_tree/find_car_avoidance_point_action.hpp"

namespace garage_utils_pkg
{

FindCarAvoidancePointAction::FindCarAvoidancePointAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<capella_ros_msg::action::FindCarAvoidancePoint>(xml_tag_name, action_name, conf)
{
}

void FindCarAvoidancePointAction::on_tick()
{
  getInput("polygons", goal_.polygons);
  getInput("car_pose", goal_.car_pose);
  getInput("car_size", goal_.car_size);
  // RCLCPP_INFO(node_->get_logger(), "on tick set state %d", garage_utils_msgs::msg::State::GENERATE_PATH);
  setOutput("state", garage_utils_msgs::msg::State::SEARCHING_PARKING_SPACE);

  // just for test
  // double rects_[][4][2] = {
  //       {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
  //       {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
  //       {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
  //       {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
  //       {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
  //       {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
  // };
  // int number = 6;

  // double car_coord[2] = {0.0 , 2.0};  // car的 x,y 坐标

  // geometry_msgs::msg::PoseStamped car_pose;
  // car_pose.header.frame_id = "map";
  // car_pose.pose.position.x = car_coord[0];
  // car_pose.pose.position.y = car_coord[1];
  
  // std::vector<geometry_msgs::msg::Polygon> polygons;
  // for (int i = 0; i < number; i++)
  // {
  //         geometry_msgs::msg::Polygon polygon;
  //         for (int j = 0; j < 4; j++)
  //         {
  //                 geometry_msgs::msg::Point32 point;
  //                 point.x = rects_[i][j][0];
  //                 point.y = rects_[i][j][1];
  //                 polygon.points.push_back(point);
  //         }
  //         polygons.push_back(polygon);
  // }

  // goal_.car_pose = car_pose;
  // goal_.polygons = polygons;
}

BT::NodeStatus FindCarAvoidancePointAction::on_success()
{
  setOutput("free_point", result_.result->pose);
  setOutput("goal", result_.result->pose);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindCarAvoidancePointAction::on_aborted()
{
  geometry_msgs::msg::PoseStamped empty_pose;
  setOutput("free_point", empty_pose);
  setOutput("goal", empty_pose);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FindCarAvoidancePointAction::on_cancelled()
{
  geometry_msgs::msg::PoseStamped empty_pose;
  setOutput("free_point", empty_pose);
  setOutput("goal", empty_pose);
  return BT::NodeStatus::SUCCESS;
}

void FindCarAvoidancePointAction::halt()
{
  geometry_msgs::msg::PoseStamped empty_pose;
  setOutput("free_point", empty_pose);
  setOutput("goal", empty_pose);
  BtActionNode::halt();
}

}  // namespace garage_utils_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<garage_utils_pkg::FindCarAvoidancePointAction>(
        name, "find_car_avoidance_point_action", config);
    };

  factory.registerBuilder<garage_utils_pkg::FindCarAvoidancePointAction>(
    "FindCarAvoidancePoint", builder);
}
