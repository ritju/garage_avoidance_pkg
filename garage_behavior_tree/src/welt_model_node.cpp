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

#include <string>
#include <memory>

#include "garage_behavior_tree/welt_model_node.hpp"

namespace garage_utils_pkg
{

WeltModelNode::WeltModelNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<capella_ros_msg::action::Welt>(xml_tag_name, action_name, conf)
{

}
void WeltModelNode::on_tick()
{
  getInput("goal_pose", goal_.goal_pose);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<garage_utils_pkg::WeltModelNode>(name, "welt_model_node", config);
    };

  factory.registerBuilder<garage_utils_pkg::WeltModelNode>("WeltModelNode", builder);
}
