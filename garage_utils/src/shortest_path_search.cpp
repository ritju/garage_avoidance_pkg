#include "garage_utils/shortest_path_search.hpp"

namespace garage_utils_pkg
{
        ShortestPathSearch::ShortestPathSearch(rclcpp::Node::SharedPtr node)
        {
                this->node_ = node;
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] ShortestPathSearch constructor."); 
        }

        ShortestPathSearch::~ShortestPathSearch()
        {
             RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] ShortestPathSearch destructor.");   
        }

        void ShortestPathSearch::process_(const std::vector<EnhancedPoint>& points, const int& start_index)
        {
                this->points_ = points;

                // 初始化节点遍历的掩码
                this->mask_ = 0;

                // 初始化 distance_和path_
                this->distance_ = 0.0;
                this->path_.clear();

                get_sub_path(this->points_, -1, start_index, distance_, path_);
        }

        std::vector<int> ShortestPathSearch::generate_path1(const std::vector<EnhancedPoint>& points)
        {
                // 1、构建邻接矩阵
                int n = points.size();
                std::vector<std::vector<double>> dist(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
                for (int i = 0; i < n; i++)
                {
                        for (int j : points[i].adjacent_vec)
                        {
                                double dx = points[i].coord.first - points[j].coord.first;
                                double dy = points[i].coord.second - points[j].coord.second;
                                dist[i][j] = std::sqrt(dx*dx + dy*dy);
                        }
                        dist[i][i] = 0;
                }
                // Floyd-Warshall算法
                for (int k = 0; k < n; ++k)
                {
                        for (int i = 0; i < n; ++i)
                        {
                                for (int j = 0; j < n; ++j)
                                {
                                        if (dist[i][j] > dist[i][k] + dist[k][j])
                                        {
                                                dist[i][j] = dist[i][k] + dist[k][j];
                                        }
                                }
                        }
                }

                // 2、动态规划
                int full_mask = (1 << n) - 1;
                // 初始化DP数组
                std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
                dp[1 << 0] = {0}; // 起点是0，初始mask只有0被访问

                for (int mask = 0; mask < (1 << n); ++mask) 
                {
                        for (int u = 0; u < n; ++u) 
                        {
                                if (!(mask & (1 << u))) continue; // 当前节点u未被访问，跳过
                                for (int v = 0; v < n; ++v) 
                                {
                                        if (mask & (1 << v)) continue; // v已经被访问过，跳过（因为每次必须访问新的节点）
                                        // 否则，尝试从u走到v，并更新新的mask
                                        int new_mask = mask | (1 << v);
                                        if (dp[new_mask][v] > dp[mask][u] + dist[u][v]) 
                                        {
                                                dp[new_mask][v] = dp[mask][u] + dist[u][v];
                                        }
                                }
                        }
                }

                // 现在寻找所有节点都被访问过，且当前节点是终点的情况的最小值
                double min_dist = std::numeric_limits<double>::infinity();
                for (int u = 0; u < n; ++u) 
                {
                        if (u == n-1)
                        {       // 当前节点必须是终点
                                if (dp[full_mask][u] < min_dist) 
                                {
                                        min_dist = dp[full_mask][u];
                                }
                        }
                }
        }

        std::vector<int> ShortestPathSearch::generate_path2(const std::vector<EnhancedPoint>& points) // 深度优先，不存在环路，多个子结点时，深度低的优先。
        {

        }

        void ShortestPathSearch::get_sub_path(const std::vector<EnhancedPoint>& points, const int& parent,  const int& current, double& distance, std::vector<int>& path)
        {
                bool is_leaf = false;
                std::stringstream ss_record_distance;
                ss_record_distance << "(init: " << distance << ") => ";
                // parent/current/child的数值 在points中对应的index是不同的
                int index_parent, index_current, index_child;
                index_parent = get_points_index(points, parent);
                index_current = get_points_index(points, current);

                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] ");
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] === get_sub_path(%d, %d) ===", parent, current);
                pre_[current] = parent;
                std::priority_queue<std::pair<double, std::vector<int>>, 
                        std::vector<std::pair<double, std::vector<int>>>, 
                        std::greater<std::pair<double, std::vector<int>>>> pq;
                
                // 1、更新distance: 增加parent到current的距离
                if (parent == -1) // 根节点
                {
                        distance += 0.0;  // index -1 到 index 0的距离
                }
                else
                {                        
                        double x_parent = points[index_parent].coord.first;
                        double y_parent = points[index_parent].coord.second;
                        double x_current = points[index_current].coord.first;
                        double y_current = points[index_current].coord.second;

                        // 更新distance
                        double distance_parent2current = std::hypot(x_current - x_parent, y_current - y_parent);
                        ss_record_distance << "(" << distance << "+=" << distance_parent2current << "*2=" ;
                        distance += distance_parent2current * 2;
                        ss_record_distance << distance << ") => ";
                }

                // 2、更新path: 添加current到path
                path.push_back(current);

                // 3、递归处理子结点
                for (int child : points[index_current].adjacent_vec)
                {
                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] ");
                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] parent : %d", parent);
                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] current: %d", current);
                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] child  : %d", child);

                        double child_dis = 0.0;
                        std::vector<int> child_path;

                        if(child == parent) 
                        {
                                if (points[index_current].adjacent_vec.size() > 1) // adjacent_vec.size() > 1, 跳过子节点是父节点的子节点。
                                {
                                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] child of current is parent, continue");
                                        continue;
                                }
                                else  // 这是一个叶节点
                                {
                                        is_leaf = true;
                                        RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] leaf: %d", current);
                                        int child = points[index_current].adjacent_vec[0];
                                        index_child = get_points_index(points, child);

                                        double x_current = points[index_current].coord.first;
                                        double y_current = points[index_current].coord.second;
                                        double x_child = points[index_child].coord.first;
                                        double y_child = points[index_child].coord.second;

                                        // 更新子节点child_dis
                                        child_dis = std::hypot(x_current - x_child, y_current - y_child);

                                        // 更新child_path
                                        // 因为当前节点是叶节点，已经没有子节点，所以只需添加当前叶节点到path,前面已经添加过，不可重复添加
                                        // 返回父节点
                                        path.push_back(parent);

                                        pq.push(std::make_pair(child_dis, child_path));
                                }                                
                        }
                        else
                        {
                                get_sub_path(points, current, child, child_dis, child_path);
                                // 添加返回current的path
                                child_path.push_back(current);

                                pq.push(std::make_pair(child_dis, child_path));
                        }
                }
                
                std::pair<double, std::vector<int>> pq_child;
                while(!pq.empty())
                {
                      pq_child = pq.top();

                      double dis_child = pq_child.first;
                      if (!is_leaf)
                      {
                        ss_record_distance << "(" << distance << "+=" << dis_child << "=" ;
                        distance += dis_child;
                        ss_record_distance << distance << ") => ";
                      }
                      std::vector<int> path_sub = pq_child.second;

                      for (int index : path_sub)
                      {
                        path.push_back(index);
                      }

                      pq.pop(); 
                }

                std::stringstream ss;
                for (size_t i = 0; i < path.size(); i++)
                {
                        ss << path[i] << " ";
                }
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] *** get_sub_path(%d, %d) ***", parent, current);
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] distance record: %s", ss_record_distance.str().c_str());
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] distance: %.3f", distance);
                RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] path    : [ %s]", ss.str().c_str());
        }

        int ShortestPathSearch::get_points_index(const std::vector<EnhancedPoint> points, int index)
        {
                for (size_t i = 0; i < points.size(); i++)
                {
                        if (points[i].index == index)
                        {
                                return (int)i;
                        }
                }

                return -1;
        }

        template<typename T>
        bool ShortestPathSearch::in_vector(std::vector<T> v, T index)
        {
                if (std::find(v.begin(), v.end(), index) != v.end())
                {
                        return true;
                }
                return false;
        }

        void ShortestPathSearch::filter_path(const std::vector<EnhancedPoint> points, std::vector<int>& path)
        {
                std::vector<int> path_new;
                size_t size = points.size();

                int mask = 0;
                int pre = -1;
                for (int current : path)
                {
                        if (mask == (1 <<size) - 1)
                        {
                                break;
                        }
                        else
                        {
                                if (pre != current)
                                {
                                        path_new.push_back(current);
                                        pre = current;

                                        int current_index = get_points_index(points, current);
                                        // RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] current : %d", current);
                                        // RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] pre mask: %d", mask);
                                        mask |= 1 << current_index;
                                        // RCLCPP_INFO(node_->get_logger(), " [shortest_path_search] suf mask: %d", mask);                     
                                }
                        }

                }

                path = path_new;
        }

        int ShortestPathSearch::get_start_point_index(double robot_x, double robot_y, const std::vector<EnhancedPoint>& points)
        {
                int index = 0;
                double dis_min = std::numeric_limits<double>::max();
                for (size_t i = 0; i < points.size(); i++)
                {
                        auto point = points[i];
                        double point_x, point_y;
                        point_x = point.coord.first;
                        point_y = point.coord.second;
                        double dis = std::hypot(point_x - robot_x, point_y - robot_y);
                        if (dis < dis_min)
                        {
                                dis_min = dis;
                                index = points[i].index;
                        }
                }                

                return index;
        }
        

} // end of namespace



