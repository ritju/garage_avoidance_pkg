#ifndef GARAGE_UTILS__SHORTEST_PATH_SEARCH_HPP_
#define GARAGE_UTILS__SHORTEST_PATH_SEARCH_HPP_

#include "garage_utils/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <queue>


namespace garage_utils_pkg
{
        
        class ShortestPathSearch
        {
        public:
                explicit ShortestPathSearch(rclcpp::Node::SharedPtr node);
                ~ShortestPathSearch();

                void process_(const std::vector<EnhancedPoint>& points);

                std::vector<int> generate_path1(const std::vector<EnhancedPoint>& points); // dijstra算法
                std::vector<int> generate_path2(const std::vector<EnhancedPoint>& points); // dfs算法
                std::vector<int> get_path()
                {
                        return path_;
                }

                // 递归函数
                void get_sub_path(const std::vector<EnhancedPoint>& points, const int& parent,  const int& current,double& distance, std::vector<int>& path);

                int get_points_index(const std::vector<EnhancedPoint> points, int index);

                template<typename T>
                bool in_vector(std::vector<T>, T index);

                void filter_path(const std::vector<EnhancedPoint> points, std::vector<int>& path);
                
        
        private:
                std::vector<EnhancedPoint> points_;
                rclcpp::Node::SharedPtr    node_;
                std::vector<int>  path_;
                double distance_;
                int mask_;   //按位 保存节点是否已经被遍历过， 0=>否， 1=>是
                std::unordered_map<int, int> pre_; // 保存每个节点的父节点。
        };
} // end of namespace

#endif





