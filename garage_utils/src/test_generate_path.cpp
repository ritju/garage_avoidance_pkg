#include "garage_utils/generate_path.hpp"
#include "garage_utils/shortest_path_search.hpp"

double points_list[11][2] =
{
        {0.0, 0.0},     // 0
        {0.0, 21.5},    // 1
        {0.0, 7.0},     // 2
        {15.0, 7.0},    // 3
        {0.0, 14.0},    // 4
        {15.0, 14.0},   // 5
        {-12.0, 21.5},  // 6
        {10.75, 21.75}, // 7
        {11.5, 33.0},   // 9
        {-12.0, 10.0},  // 10
        {-12.0, 30.0}   // 11
} ;

int points_index[11] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11};

int adjacent[11][11] = 
{
       //0  1  2  3  4  5  6  7  9  10 11
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},    // 0 -> 2
        {0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0},    // 1 -> 4 6 7
        {1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},    // 2 -> 0 3 4
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},    // 3 -> 2
        {0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0},    // 4 -> 1 2 5
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    // 5 -> 4
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1},    // 6 -> 1 10 11
        {0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0},    // 7 -> 1 9
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},    // 9 -> 7
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},    // 10 -> 6
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},    // 11 -> 6
};

double rects_[][4][2] = {
        {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
        {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
        {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
        {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
        {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
        {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
};

int number = 6;

int main(int argc, char**argv)
{
        rclcpp::init(argc, argv);

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

        std::vector<garage_utils_pkg::EnhancedPoint> e_points;
        for (int i = 0; i < 11; i++)
        {
                garage_utils_pkg::EnhancedPoint e_point;
                e_point.index = points_index[i];
                std::pair<double, double> coord;
                coord.first = points_list[i][0];
                coord.second = points_list[i][1];
                e_point.coord = coord;
                e_point.visited = false;
                for (int j = 0; j < 11; j++)
                {
                        if (adjacent[i][j] == 1)
                        {
                                e_point.adjacent_vec.push_back(points_index[j]);
                        }                        
                }
                e_points.push_back(e_point);
        }
        
        auto node = std::make_shared<rclcpp::Node>("test_generate_path");

        try
        {
               auto shortest_path_searcher = new garage_utils_pkg::ShortestPathSearch(node);
               shortest_path_searcher->process_(e_points);
               auto order = shortest_path_searcher->get_path();
               shortest_path_searcher->filter_path(e_points, order);

               std::stringstream ss;
                for (size_t i = 0; i < order.size(); i++)
                {
                        ss << order[i] << " ";
                }
                RCLCPP_INFO(node->get_logger(), "path => %s", ss.str().c_str());
               
               std::vector<garage_utils_pkg::EnhancedPoint> points_ordered;
               for(size_t i = 0; i < order.size(); i++)
               {
                garage_utils_pkg::EnhancedPoint e_point;
                for (size_t j = 0; j < e_points.size(); j++)
                {
                        if (e_points[j].index == order[i])
                        {
                                e_point = e_points[j];
                                RCLCPP_INFO(node->get_logger(), "order: %zd", order[i]);
                                RCLCPP_INFO(node->get_logger(), "points => index: %d, coord: (%f, %f)",
                                        e_points[j].index, e_points[j].coord.first, e_points[j].coord.second);
                                points_ordered.push_back(e_point);
                                break;
                        }
                }           
               }

               auto path_generator = new garage_utils_pkg::GeneratePath(node);
               path_generator->process_(points_ordered, polygons);
               auto path = path_generator->get_path();
               auto path_pub = node->create_publisher<nav_msgs::msg::Path>("garage_path", rclcpp::QoS(1).reliable());
               
               double time_elapse = 0.0;
               double time_delta = 0.1;
               while(time_elapse < 5.0)
               {
                        RCLCPP_INFO_THROTTLE(node->get_logger(),*(node->get_clock()), 1000, "time_elapsed: %f", time_elapse);
                        path_pub->publish(path);
                        std::this_thread::sleep_for(std::chrono::duration<double>(time_delta));
                        time_elapse += time_delta;
               }
        }
        catch(const std::string & e)
        {
                std::cerr << e.c_str() << '\n';
        }
        catch (const std::exception & e)
        {
                std::cerr << e.what() << "\n";
        }


        rclcpp::spin(node);
        rclcpp::shutdown();
}









