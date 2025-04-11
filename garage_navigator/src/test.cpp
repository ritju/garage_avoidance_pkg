#include "garage_utils_msgs/action/garage_vehicle_avoidance.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "visualization_msgs/msg/marker_array.hpp"


// double rects_[][4][2] = {
//         {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
//         {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
//         {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
//         {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
//         {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
//         {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
// };
// int number = 6;

double rects_[][4][2] = {
        {{4.5, -6.0}, {4.0, 0.0}, {2.0, 0.0}, {2.5, -6.0}},
        {{2.0, 0.0}, {-5.0, 0.0}, {-5.0, -2.0}, {2.0, -2.0}}
};
int number = 2;

double car_coord[2] = {3.0 , -7.0};   // car的 x,y 坐标
double car_size[3] = {1.0, 0.5, 1.0}; // car的size

rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr goal_handle_;

void goal_response_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr& goal_handle)
{
        std::cout << "goal response callback" << std::endl;
        goal_handle_ = goal_handle;
        if (!goal_handle_) 
        {
                std::cout << "Goal was rejected by server" << std::endl;
        } 
        else 
        {
                std::cout << "Goal accepted by server, waiting for result" << std::endl;
        }
}

void feedback_callback(rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr goal_handle, const std::shared_ptr<const garage_utils_msgs::action::GarageVehicleAvoidance::Feedback> feedback)
{
        (void)goal_handle;

        (void)feedback;
        // std::cout << "state " << (int)feedback->state << std::endl;
}

void result_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::WrappedResult& result)
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
        std::cout << "reason: " << result.result->reason << std::endl;
}

int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);

        // define node
        auto node = std::make_shared<rclcpp::Node>("action_client");

        // show rects in rviz2 by visualization_msgs
        auto rects_pub = node->create_publisher<visualization_msgs::msg::Marker>("rects_visualization", rclcpp::QoS(5).reliable().transient_local());
        visualization_msgs::msg::Marker msg;
        msg.header.frame_id = "map";
        msg.header.stamp = node->now();
        msg.ns = "test";
        msg.id = 0;
        msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        msg.action = 0;
        msg.scale.x = 0.1;
        msg.color.r = 0.0;
        msg.color.g = 0.0;
        msg.color.b = 1.0;
        msg.color.a = 1.0;
        for (int i = 0; i < number; i++)
        {                
                for (int j = 0; j < 4; j++)
                {
                        geometry_msgs::msg::Point p_start;
                        p_start.x = rects_[i][j][0];
                        p_start.y = rects_[i][j][1];
                        p_start.z = 0.0;
                        msg.points.push_back(p_start);
                        geometry_msgs::msg::Point p_end;
                        p_end.x = rects_[i][(j+1)%4][0];
                        p_end.y = rects_[i][(j+1)%4][1];
                        p_end.z = 0.0;
                        msg.points.push_back(p_end);
                }
        }
        // for (int i = 0; i < 5; i++)
        // {
        //         RCLCPP_INFO(node->get_logger(), "publish rects visualization.");
                rects_pub->publish(msg);
        //         std::this_thread::sleep_for(std::chrono::seconds(1));
        // }
        
        

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
        auto client_ptr_ = rclcpp_action::create_client<garage_utils_msgs::action::GarageVehicleAvoidance>(
                node, "/garage_vehicle_avoidance");

        using namespace std::chrono_literals;
        while (!client_ptr_->wait_for_action_server(1s)) 
        {
                RCLCPP_WARN(node->get_logger(), "Action server not available, waiting");
                std::this_thread::sleep_for(std::chrono::duration<double>(1.0));                
        }

        auto goal_msg = garage_utils_msgs::action::GarageVehicleAvoidance::Goal();
        capella_ros_msg::msg::CarDetectSingle car_information;
        geometry_msgs::msg::Vector3 size;
        size.x = car_size[0];
        size.y = car_size[1];
        size.z = car_size[2];
        car_information.pose = car_pose;
        car_information.size = size;
        goal_msg.cars_information.results.push_back(car_information);
        goal_msg.polygons = polygons;
        
        RCLCPP_INFO(node->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<garage_utils_msgs::action::GarageVehicleAvoidance>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.result_callback = result_callback;
        send_goal_options.feedback_callback = feedback_callback;

        auto future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // test for cancel goal
        std::thread([&](){
                std::this_thread::sleep_for(std::chrono::seconds(3));
                if (goal_handle_)
                {
                        RCLCPP_INFO(node->get_logger(), "cancel the goal.");
                        auto cancel_future = client_ptr_->async_cancel_goal(goal_handle_);
                        cancel_future.wait();
                        RCLCPP_INFO(node->get_logger(), "cancel goal completed.");
                }
                else
                {
                        RCLCPP_INFO(node->get_logger(), "goal_handle is empty ptr, failed to cancel goal."); 
                }   
                }).detach();
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        // rclcpp::spin(node);
        rclcpp::shutdown();
}
