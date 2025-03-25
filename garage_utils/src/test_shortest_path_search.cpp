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

int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("test_shortest_path_search");

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

        try
        {
                auto path_searcher = new garage_utils_pkg::ShortestPathSearch(node);
                path_searcher->process_(e_points);
                
                auto path = path_searcher->get_path();
                path_searcher->filter_path(e_points, path);
                
                std::stringstream ss;
                for (size_t i = 0; i < path.size(); i++)
                {
                        ss << path[i] << " ";
                }
                RCLCPP_INFO(node->get_logger(), "path => %s", ss.str().c_str());
        }
        catch(const std::string & e)
        {
                std::cerr << e.c_str() << '\n';
        }
        catch (const std::exception & e)
        {
                std::cerr << e.what() << "\n";
        }

        rclcpp::shutdown();
}




