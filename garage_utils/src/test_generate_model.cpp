#include "garage_utils/generate_model.hpp"

#include "algorithm"
#include <random>


// [0-n) 随即排序
std::vector<int> generateRandomPermutation(int n) 
{
    std::vector<int> permutation;
    for (int i = 0; i <= n; ++i) {
        permutation.push_back(i);
    }
    
    // 使用随机设备作为种子初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 打乱数组顺序
    std::shuffle(permutation.begin(), permutation.end(), gen);
    
    return permutation;
}

// [0-n) 全排列
std::vector<std::vector<int>> generateAllPermutations(int n) 
{
    std::vector<std::vector<int>> result;
    std::vector<int> nums;
    
    // 初始化原始序列 [0,1,2,...,n-1]
    for (int i = 0; i < n; ++i) {
        nums.push_back(i);
    }
    
    // 使用STL算法生成全排列
    do {
        result.push_back(nums);  // 记录当前排列
    } while (std::next_permutation(nums.begin(), nums.end()));
    
    return result;
}

double dis_thr = 2.5;

int number = 6;

// double rects_[][4][2] = {
//         {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
//         {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
//         {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
//         {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
//         {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
//         {{-13.0, 10.0}, {-11.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}}
// };

double rects_[][4][2] = {
        {{1.0, 0.0}, {1.0, 20.0}, {-1.0, 20.0}, {-1.0, 0.0}},
        {{1.5, 6.0}, {15.0, 6.0}, {15, 8.0}, {1.5, 8.0}},
        {{1.5, 13.0}, {15.0, 13.0}, {15, 15.0}, {1.5, 15.0}},
        {{-10.0, 20.5}, {10.0, 20.5}, {10.0, 22.5}, {-10, 22.5}},
        {{10.5, 22.0}, {12.5, 22.0}, {12.5, 33.0}, {10.5, 33.0}},
        {{-13.0, 10.0}, {-11.0, 30.0}, {-13.0, 30.0}, {-11.0, 10.0}}  // 打乱顺序
};




int main(int argc, char** argv)
{
        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("test_generate_model");

        try
        {
                // 1、固定次数的随即排列
                // for (int i = 0; i < 10; i++)
                // {
                //         auto permutation = generateRandomPermutation(number - 1);
                //         std::stringstream ss;
                //         ss << "[ ";
                //         for (int j = 0; j < number; j++)
                //         {
                //                 ss << permutation[j];
                //                 if (j == number - 1)
                //                 {
                //                         ss << " ";
                //                 }
                //                 else
                //                 {
                //                         ss << ", "; 
                //                 }
                //         }
                //         ss << "]";
                //         RCLCPP_INFO(node->get_logger(), "permutation: %s", ss.str().c_str());
                // }

                // 2、[0-n) 全排列

                auto permutations = generateAllPermutations(number);
                RCLCPP_INFO(node->get_logger(), "permutations size: %zd", permutations.size());

                // 打印全排列的所有组合
                // for (size_t i = 0; i < permutations.size(); i++)
                // {
                //         std::stringstream ss_permutation;
                //         ss_permutation << "index " << i << " => [ ";
                //         for (size_t j = 0; j < permutations[i].size(); j++)
                //         {
                //                ss_permutation << permutations[i][j];
                //                if (j == permutations[i].size() - 1)
                //                {
                //                 ss_permutation << " ";
                //                } 
                //                else
                //                {
                //                 ss_permutation << ", ";
                //                }
                //         }
                //         ss_permutation << "]";
                //         RCLCPP_INFO(node->get_logger(), "%s", ss_permutation.str().c_str());
                // }

                for (size_t p_i = 0; p_i < permutations.size(); p_i++)
                {
                        std::stringstream ss_permutation;
                        ss_permutation << "index " << p_i << " => [ ";
                        for (size_t j = 0; j < permutations[p_i].size(); j++)
                        {
                               ss_permutation << permutations[p_i][j];
                               if (j == permutations[p_i].size() - 1)
                               {
                                ss_permutation << " ";
                               } 
                               else
                               {
                                ss_permutation << ", ";
                               }
                        }
                        ss_permutation << "]";
                        RCLCPP_INFO(node->get_logger(), "current permutation: %s", ss_permutation.str().c_str());
                        
                        std::vector<std::vector<Point>> rects;
                        for (int i = 0; i < number; i++)
                        {
                                std::vector<Point> p_v;
                                for (int j = 0; j < 4; j++)
                                {
                                        Point pt;
                                        // pt.first = rects_[i][j][0];
                                        // pt.second = rects_[i][j][1];
                                        pt.first = rects_[permutations[p_i][i]][j][0];
                                        pt.second = rects_[permutations[p_i][i]][j][1];
                                        p_v.push_back(pt);
                                }
                                rects.push_back(p_v);
                        }

                        // RCLCPP_INFO(node->get_logger(), "rects size: %zd", rects.size());
                        
                        auto model_generator = new garage_utils_pkg::GenerateModel(rects, node, dis_thr);
                        auto points = model_generator->get_points();

                        
                        RCLCPP_INFO(node->get_logger(), "print result");
                        RCLCPP_INFO(node->get_logger(), "points size: %zd", points.size());
                        for (size_t i = 0; i < points.size(); i++)
                        {
                                auto point = points[i];
                                RCLCPP_INFO(node->get_logger(), "---------- points %zd ----------", i);
                                RCLCPP_INFO(node->get_logger(), "index: %d", point.index);
                                RCLCPP_INFO(node->get_logger(), "visited: %s", point.visited?"true":"false");
                                RCLCPP_INFO(node->get_logger(), "coord: (%f, %f)", point.coord.first, point.coord.second);
                                std::stringstream ss;
                                // RCLCPP_INFO(node->get_logger(), "adjacent_vec size: %zd", point.adjacent_vec.size());
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
                                RCLCPP_INFO(node->get_logger(), "adjacent: [ %s]", ss.str().c_str());
                        }
                }                
        }
        catch(const std::string& e)
        {
                std::cerr << e.c_str() << '\n';
        }
        

        rclcpp::shutdown();
}


