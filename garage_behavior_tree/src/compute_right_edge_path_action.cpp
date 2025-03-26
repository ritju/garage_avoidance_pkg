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

#include "garage_behavior_tree/compute_right_edge_path_action.hpp"

namespace garage_utils_pkg
{

ComputeRightEdgePathAction::ComputeRightEdgePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<garage_utils_msgs::action::ComputeRightEdgePath>(xml_tag_name, action_name, conf)
{
}

void ComputeRightEdgePathAction::on_tick()
{
  getInput("goal", goal_.polygons);
}

BT::NodeStatus ComputeRightEdgePathAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeRightEdgePathAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRightEdgePathAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

void ComputeRightEdgePathAction::halt()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  BtActionNode::halt();
}

}  // namespace garage_utils_pkg

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<garage_utils_pkg::ComputeRightEdgePathAction>(
        name, "compute_right_edge_path_action_server", config);
    };

  factory.registerBuilder<garage_utils_pkg::ComputeRightEdgePathAction>(
    "ComputeRightEdgePath", builder);
}
