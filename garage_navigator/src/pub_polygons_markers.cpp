#include "rclcpp/rclcpp.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

garage_utils_msgs::msg::Polygons polygons_msg;

void polygons_sub_callback()
{
        
}

int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);        

        auto node = std::make_shared<rclcpp::Node>("pub_polygons_markers_node");

        auto polygons_markers_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("rects_visualization", rclcpp::QoS(5).reliable().transient_local());

        auto callback_type = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_type;
        auto polygons_sub_ = node->create_subscription<garage_utils_msgs::msg::Polygons>("garage_polygons", 
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), [&](const garage_utils_msgs::msg::Polygons::SharedPtr msgs)
                {
                        RCLCPP_INFO(node->get_logger(), "received a msg for /garage_polygons.");
                        polygons_msg = *msgs;

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
                        for (int i = 0; i < (int)polygons_msg.polygons.size(); i++)
                        {                
                                for (int j = 0; j < 4; j++)
                                {
                                        geometry_msgs::msg::Point p_start;
                                        p_start.x = polygons_msg.polygons[i].points[j].x;
                                        p_start.y = polygons_msg.polygons[i].points[j].y;
                                        p_start.z = 0.0;
                                        msg.points.push_back(p_start);
                                        geometry_msgs::msg::Point p_end;
                                        p_end.x = polygons_msg.polygons[i].points[(j+1)%4].x;
                                        p_end.y = polygons_msg.polygons[i].points[(j+1)%4].y;
                                        p_end.z = 0.0;
                                        msg.points.push_back(p_end);
                                }
                        }
                        RCLCPP_INFO(node->get_logger(), "pub polygons_marker msg.");
                        polygons_markers_pub_->publish(msg);
                }, sub_options);

        

        rclcpp::spin(node);
        rclcpp::shutdown();

}

