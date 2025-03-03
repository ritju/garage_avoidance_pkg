#ifndef GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_H_
#define GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_H_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "garage_utils_msgs/action/compute_right_edge_path.hpp"

namespace garage_utils_pkg
{
class ComputeRightEdgePathActionServer : public rclcpp::Node
{
public:
  using ComputeRightEdgePath = garage_utils_msgs::action::ComputeRightEdgePath;
  using GoalHandleComputeRightEdgePath = rclcpp_action::ServerGoalHandle<ComputeRightEdgePath>;

  explicit ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ComputeRightEdgePathActionServer();

private:
  rclcpp_action::Server<ComputeRightEdgePath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeRightEdgePath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

  void execute(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

};  // class ComputeRightEdgePathActionServer

}  // namespace garage_utils_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(garage_utils_pkg::ComputeRightEdgePathActionServer)


#endif