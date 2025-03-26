#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "garage_utils/types.hpp"

#include "garage_utils_msgs/action/compute_right_edge_path.hpp"

#include <chrono>

double rects_[][4][2] = {
        {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
        {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
        {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
        {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
        {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
        {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
};
int number = 6;

double car_coord[2] = {0.0 , 2.0};  // car的 x,y 坐标

void goal_response_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::ComputeRightEdgePath>::SharedPtr& goal_handle)
{
        if (!goal_handle) 
        {
                std::cout << "Goal was rejected by server" << std::endl;
        } 
        else 
        {
                std::cout << "Goal accepted by server, waiting for result" << std::endl;
        }
}

void feedback_callback(rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::ComputeRightEdgePath>::SharedPtr goal_handle, const std::shared_ptr<const garage_utils_msgs::action::ComputeRightEdgePath::Feedback> feedback)
{
        (void)goal_handle;
        (void)feedback;
}

void result_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::ComputeRightEdgePath>::WrappedResult& result)
{
        switch (result.code) 
        {
                case rclcpp_action::ResultCode::SUCCEEDED:
                std::cout << "Goal was succeed." << std::endl;
                
                break;

                case rclcpp_action::ResultCode::ABORTED:
                std::cout << "Goal was aborted" << std::endl;
                return;

                case rclcpp_action::ResultCode::CANCELED:
                std::cout << "Goal was canceled" << std::endl;
                return;

                default:
                std::cout << "Unknown result code" << std::endl;
                return;
        }

        auto path = result.result->path;
        std::cout << "path size: " << path.poses.size() << std::endl;
}

int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);

        // define node
        auto node = std::make_shared<rclcpp::Node>("action_client");

        geometry_msgs::msg::PoseStamped car_pose;
        car_pose.header.frame_id = "map";
        car_pose.pose.position.x = car_coord[0];
        car_pose.pose.position.y = car_coord[1];
        
        std::vector<geometry_msgs::msg::Polygon> polygons;
        for (int i = 0; i < number; i++)
        {
                geometry_msgs::msg::Polygon polygon;
                for (int j = 0; j < 4; j++)
                {
                        geometry_msgs::msg::Point32 point;
                        point.x = rects_[i][j][0];
                        point.y = rects_[i][j][1];
                        polygon.points.push_back(point);
                }
                polygons.push_back(polygon);
        }
        
        RCLCPP_INFO(node->get_logger(), "create action client ...");
        auto client_ptr_ = rclcpp_action::create_client<garage_utils_msgs::action::ComputeRightEdgePath>(
                node, "compute_right_edge_path_action_server");

        using namespace std::chrono_literals;
        while (!client_ptr_->wait_for_action_server(1s)) 
        {
                RCLCPP_WARN(node->get_logger(), "Action server not available, waiting");
                std::this_thread::sleep_for(std::chrono::duration<double>(1.0));                
        }

        auto goal_msg = garage_utils_msgs::action::ComputeRightEdgePath::Goal();
        goal_msg.car_pose = car_pose;
        goal_msg.polygons = polygons;
        
        RCLCPP_INFO(node->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<garage_utils_msgs::action::ComputeRightEdgePath>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.result_callback = result_callback;
        send_goal_options.feedback_callback = feedback_callback;

        client_ptr_->async_send_goal(goal_msg, send_goal_options);        
        
        
        rclcpp::spin(node);
        rclcpp::shutdown();
}