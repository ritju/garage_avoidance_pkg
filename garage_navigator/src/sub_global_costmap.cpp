#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

void sub_global_costmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
        size_t size = msg->data.size();

        int count_0 = 0, count_1 = 0, count_minus_1 = 0, count_100 = 0, count_99 = 0, count_255 = 0, count_254 = 0, count_more_than_100 = 0, count_less_and_equal_than_100 = 0;

        for (size_t i = 0; i < size; i++)
        {
           auto value = (int)msg->data[i];
           switch(value)
           {
                case 0:
                        count_0++;
                        break;

                case 1:
                        count_1++;
                        break;

                case -1:
                        count_minus_1++;
                        break;

                case 100:
                        count_100++;
                        break;

                case 99:
                        count_99++;
                        break;

                case 255:
                        count_255++;
                        break;
                
                case 254:
                        count_254++;
                        break;
           }

           if (value > 100)
           {
                count_more_than_100++;
           }
           else
           {
                count_less_and_equal_than_100++;
           }
        }
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "-------------------");
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_0    : %d", count_0);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_1    : %d", count_1);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_-1   : %d", count_minus_1);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_99   : %d", count_99);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_100  : %d", count_100);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_254  : %d", count_254);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_255  : %d", count_255);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_>100 : %d", count_more_than_100);
        RCLCPP_INFO(rclcpp::get_logger("test_costmap"), "count_<=100: %d", count_less_and_equal_than_100);
}

int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("test_costmap");

        auto sub_global_costmap_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(), sub_global_costmap_callback);

        rclcpp::spin(node);
        rclcpp::shutdown();
}


