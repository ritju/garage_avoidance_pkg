#include "garage_utils/generate_model.hpp"


double dis_thr = 2.0;

int number = 6;

// double points[][4][2] = {
//         {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
//         {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
//         {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
//         {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
//         {{11.0, 23.0}, {13.0, 23.0}, {13.0, 33.0}, {11.0, 33.0}},
//         {{-13.0, 10.0}, {-11.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}}
// };

double points[][4][2] = {
        {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
        {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
        {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
        {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
        {{11.0, 23.0}, {13.0, 23.0}, {13.0, 33.0}, {11.0, 33.0}},
        {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
};


int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("test_generate_model");

        try
        {
                std::vector<std::vector<Point>> rects;
                for (int i = 0; i < number; i++)
                {
                        std::vector<Point> p_v;
                        for (int j = 0; j < 4; j++)
                        {
                                Point pt;
                                pt.first = points[i][j][0];
                                pt.second = points[i][j][1];
                                p_v.push_back(pt);
                        }
                        rects.push_back(p_v);
                }

                // RCLCPP_INFO(node->get_logger(), "rects size: %zd", rects.size());
                
                auto model_generator = new garage_utils_pkg::GenerateModel(rects, node, dis_thr);
                auto points = model_generator->get_points();

                

                for (size_t i = 0; i < points.size(); i++)
                {
                        auto point = points[i];
                        RCLCPP_INFO(node->get_logger(), "---------- points %zd ----------", i);
                        RCLCPP_INFO(node->get_logger(), "index: %d", point.index);
                        RCLCPP_INFO(node->get_logger(), "visited: %s", point.visited?"true":"false");
                        RCLCPP_INFO(node->get_logger(), "coord: (%f, %f)", point.coord.first, point.coord.second);
                        std::stringstream ss;
                        for (size_t j = 0; j < point.adjacent_vec.size(); j++)
                        {
                                ss << point.adjacent_vec[j] ;
                                if (j == point.adjacent_vec.size() - 1)
                                {
                                        ss << " ";
                                }
                                else
                                {
                                        ss << ", ";
                                }
                        }
                        RCLCPP_INFO(node->get_logger(), "adjacent: [ %s]", point.coord.first, point.coord.second);
                }   
        }
        catch(const std::string& e)
        {
                std::cerr << e.c_str() << '\n';
        }
        

        rclcpp::shutdown();
}


