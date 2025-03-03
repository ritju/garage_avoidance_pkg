#include "garage_utils/compute_right_edge_path.hpp"

namespace garage_utils_pkg
{
        ComputeRightEdgePathActionServer::ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options)
                : Node("compute_right_edge_path_action_server", options)
        {
                using namespace std::placeholders;

                this->action_server_ = rclcpp_action::create_server<ComputeRightEdgePath>(
                this,
                "compute_right_edge_path_action_server",
                std::bind(&ComputeRightEdgePathActionServer::handle_goal, this, _1, _2),
                std::bind(&ComputeRightEdgePathActionServer::handle_cancel, this, _1),
                std::bind(&ComputeRightEdgePathActionServer::handle_accepted, this, _1));
        }
        
        ComputeRightEdgePathActionServer::~ComputeRightEdgePathActionServer()
        {

        }

        rclcpp_action::GoalResponse ComputeRightEdgePathActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const ComputeRightEdgePath::Goal> goal)
        {
                RCLCPP_INFO(this->get_logger(), "Received goal request with frequency %f and polygon's size: %ld", goal->frequency, goal->polygons.size());
                std::stringstream ss;
                geometry_msgs::msg::Polygon polygon;
                geometry_msgs::msg::Point32 point;
                ss << "[";
                for (size_t i = 0; i < goal->polygons.size(); i++)
                {
                        polygon = goal->polygons[i];
                        std::stringstream ss;
                        for (size_t j = 0; j < polygon.points.size(); j++)
                        {
                                geometry_msgs::msg::Point32 point = polygon.points[j];
                                ss << "(" <<  point.x << ", " << point.y << ")";
                                if (j != (polygon.points.size() - 1))
                                {
                                        ss << ", ";
                                }
                        }
                }
                ss << "]";
                RCLCPP_INFO(get_logger(), "polygons: %s", ss.str().c_str());
                
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse ComputeRightEdgePathActionServer::handle_cancel(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
        }

        void ComputeRightEdgePathActionServer::handle_accepted(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                using namespace std::placeholders;
                // this needs to return quickly to avoid blocking the executor, so spin up a new thread
                std::thread{std::bind(&ComputeRightEdgePathActionServer::execute, this, _1), goal_handle}.detach();
        }

        void ComputeRightEdgePathActionServer::execute(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {

        }

}
