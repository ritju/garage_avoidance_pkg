#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "capella_ros_msg/action/find_car_avoidance_point.hpp"
#include "capella_ros_msg/msg/car_detect_array.hpp"

std::vector<geometry_msgs::msg::Polygon> polygons;
bool polygons_received = false;

rclcpp::Node::SharedPtr node;
rclcpp_action::Client<capella_ros_msg::action::FindCarAvoidancePoint>::SharedPtr find_point_client;
capella_ros_msg::action::FindCarAvoidancePoint::Goal goal_msg;
rclcpp_action::Client<capella_ros_msg::action::FindCarAvoidancePoint>::SendGoalOptions send_goal_options;

void timer_callback()
{
        if(!find_point_client->wait_for_action_server())
        {
                RCLCPP_INFO(node->get_logger(), "waiting for /find_car_avoidance_point_action action ...");
                return;
        }   
        if (polygons_received)
        {
                RCLCPP_INFO(node->get_logger(), "sending_goal ...");
                goal_msg = capella_ros_msg::action::FindCarAvoidancePoint::Goal();
                geometry_msgs::msg::PoseStamped car_pose;
                car_pose.header.frame_id = "map";
                car_pose.pose.position.x = -6.0;
                car_pose.pose.position.y = -1.0;
                goal_msg.car_pose = car_pose;
                goal_msg.polygons = polygons;
                find_point_client->async_send_goal(goal_msg, send_goal_options);
        }
        else
        {
                RCLCPP_INFO(node->get_logger(), "waiting for topic /garage_polygons ...");
        }
}

void polygons_sub_callback(const garage_utils_msgs::msg::Polygons::SharedPtr msg)
{
        polygons = msg->polygons;
        RCLCPP_INFO(rclcpp::get_logger("test_find_point"), "topic /garage_polygons msg received. size is %ld", polygons.size());
        polygons_received = true;
}

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);
        node = std::make_shared<rclcpp::Node>("test_find_point");
        RCLCPP_INFO(node->get_logger(), "test find_point.");

        auto cb_type1 = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options1 = rclcpp::SubscriptionOptions();
        sub_options1.callback_group = cb_type1;
        auto polygons_sub = node->create_subscription<garage_utils_msgs::msg::Polygons>(
                "garage_polygons",
                rclcpp::QoS(1).reliable().transient_local(),
                polygons_sub_callback,
                sub_options1
        );

        auto callback_group_action_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        find_point_client = rclcpp_action::create_client<capella_ros_msg::action::FindCarAvoidancePoint>(
                node, "/find_car_avoidance_point_action", callback_group_action_);
        

        send_goal_options = rclcpp_action::Client<capella_ros_msg::action::FindCarAvoidancePoint>::SendGoalOptions();
        send_goal_options.goal_response_callback = [&](const rclcpp_action::ClientGoalHandle<capella_ros_msg::action::FindCarAvoidancePoint>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
                } else {
                        RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
                }
        };
        send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<capella_ros_msg::action::FindCarAvoidancePoint>::WrappedResult & result) {
                switch (result.code) 
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node->get_logger(), "Goal succeeded!");
                        break;
                case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
                        break;
                case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
                        break;
                default:
                        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
                        break;
                }
        };  

        auto timer_send_goal = node->create_wall_timer(0.5s, timer_callback);         

        // RCLCPP_INFO(node->get_logger(), "sending goal ...");
        // RCLCPP_INFO(node->get_logger(), "car_pose: (%f, %f)", goal_msg.car_pose.pose.position.x, goal_msg.car_pose.pose.position.y);
        // RCLCPP_INFO(node->get_logger(), "polygons size: %ld", goal_msg.polygons.size());

         
        // rclcpp::executors::MultiThreadedExecutor executors;
        // executors.add_callback_group(cb_type1, node->get_node_base_interface());
        // executors.spin();

        rclcpp::spin(node);
        rclcpp::shutdown();
}
