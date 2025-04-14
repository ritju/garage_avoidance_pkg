#ifndef GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_HPP_
#define GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <utility>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "garage_utils_msgs/action/compute_right_edge_path.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "garage_utils/types.hpp"
#include "garage_utils/generate_model.hpp"
#include "garage_utils/shortest_path_search.hpp"
#include "garage_utils/generate_path.hpp"

namespace garage_utils_pkg
{
    
class ComputeRightEdgePathActionServer : public rclcpp::Node
{
public:
  using ComputeRightEdgePath = garage_utils_msgs::action::ComputeRightEdgePath;
  using GoalHandleComputeRightEdgePath = rclcpp_action::ServerGoalHandle<ComputeRightEdgePath>;

  explicit ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ComputeRightEdgePathActionServer();

  size_t get_current_polygon_index(double x, double y, std::vector<geometry_msgs::msg::Polygon> polygons);
  bool get_map_robot_tf();
  
    // 检查goal的合法性
    bool goal_checker(std::shared_ptr<const ComputeRightEdgePath::Goal> goal, std::string& reason);

    void print_polygons(const std::vector<geometry_msgs::msg::Polygon> polygons);
    void print_rect(const EnhancedRect& rect);

    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputeRightEdgePath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

  void execute(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle);

  double distance(double x1, double y1, double x2, double y2)
  {     
          return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
  }
  double area(double x1, double y1, double x2, double y2, double x3, double y3)
  {
          return fabs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
  }

  double calculate_height(double x1, double y1, double x2, double y2, double x3, double y3)
  {
          return area(x1, y1, x2, y2, x3, y3) / distance(x2, y2, x3, y3);
  }


// 计算向量右侧方向（右手法则）
std::pair<double, double> getRightNormalVector(double dx, double dy) 
{
    return { dy, -dx }; // 右侧法向量公式‌
}

// 选择当前行进方向右侧边‌
int selectRightEdge(const EnhancedRect& rect, 
                    const std::pair<double, double>& direction) 
{
    double max_dot = -INFINITY;
    int selected_edge = -1;

    for (int i = 0; i < rect.edges.size(); ++i) 
    {
        // 计算边方向向量
        const auto& [start_idx, end_idx] = rect.edges[i];
        double dx = rect.vertices[end_idx].x - rect.vertices[start_idx].x;
        double dy = rect.vertices[end_idx].y - rect.vertices[start_idx].y;

        // 计算右侧法向量与行进方向的匹配度‌:
        auto right_normal = getRightNormalVector(dx, dy);
        double dot = right_normal.first * direction.first 
                   + right_normal.second * direction.second;

        if (dot > max_dot) {
            max_dot = dot;
            selected_edge = i;
        }
    }
    return selected_edge;
}

  // 判断size_t变量是否存在与std::vector<size_t>中
  bool existsInVector(size_t var, std::vector<size_t> vec)
  {
    return std::find(vec.begin(), vec.end(), var) != vec.end();
  }

  
  // 找到某一个点在线段上离它最近的点  
  std::pair<double, double> find_neareast_point(double px, double py, double seg1x, double seg1y, double seg2x, double seg2y)
  {
    // 计算线段方向向量分量
    double dx = seg2x - seg1x;
    double dy = seg2y - seg1y;
    
    // 处理线段退化为点的情况‌
    if (dx == 0.0 && dy == 0.0) {
        return {seg1x, seg1y};
    }
    
    // 计算点P到线段起点A的向量分量
    double apx = px - seg1x;
    double apy = py - seg1y;
    
    // 计算线段AB长度的平方和向量点积‌
    double ab2 = dx * dx + dy * dy;
    double ap_dot_ab = apx * dx + apy * dy;
    
    // 计算投影参数t并约束范围[0,1]
    double t = ap_dot_ab / ab2;
    t = std::max(0.0, std::min(1.0, t)); // 限制t在0~1之间
    
    // 根据t值计算最近点坐标‌
    double nearest_x = seg1x + t * dx;
    double nearest_y = seg1y + t * dy;
    
    return {nearest_x, nearest_y}; // 返回pair结构‌:
  }
  
  std::vector<std::pair<double, double>> offset(double dis, std::pair<double, double> pt1, std::pair<double, double> pt2) 
  {
    // 计算线段方向向量
    double dx = pt2.first - pt1.first;
    double dy = pt2.second - pt1.second;
    
    // 计算左方向的单位法向量（逆时针旋转90度）
    double length = std::sqrt(dx*dx + dy*dy);
    double unit_x = -dy / length;  // 左方向x分量‌
    double unit_y = dx / length;   // 左方向y分量‌
    
    // 计算实际偏移量
    double offset_x = unit_x * dis;
    double offset_y = unit_y * dis;
    
    // 生成新端点
    std::pair<double, double> new_pt1 = { 
        pt1.first + offset_x, 
        pt1.second + offset_y 
    };
    std::pair<double, double> new_pt2 = { 
        pt2.first + offset_x, 
        pt2.second + offset_y 
    };
    
    return {new_pt1, new_pt2};  // 返回包含两个点的vector‌
  } 


  bool isPointOnSegment(double px, double py, double x1, double y1, double x2, double y2);

  bool isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon);

  // 求解一个点到一条线段的距离：判断点到直线的垂线是否在线段上 1、在，返回点到直线的距离；2、不在，返回点到线段的两个端点的距离的平均值。
  double point_to_line_distance_smart(double point_x, double point_y, double line_pt1_x, double line_pt1_y, double line_pt2_x, double line_pt2_y);

  void init_params();

  double theta_between_two_edges(Point seg1_pt1, Point seg1_pt2, Point seg2_pt1, Point seg2_pt2);

private:
  rclcpp_action::Server<ComputeRightEdgePath>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Path path_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;

  std::vector<geometry_msgs::msg::Polygon> polygons_;
  geometry_msgs::msg::PoseStamped car_pose_;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform map_robot_tf;

  GenerateModel * model_generator_;
  ShortestPathSearch * path_searcher_;
  GeneratePath * path_generator_;

  std::vector<EnhancedRect> rects_origin;
  std::vector<EnhancedRect> rects_ordered;

  // params
  std::string path_topic_name_;
  double dis_thr_;
  bool test{false};
  double robot_x_test;
  double robot_y_test;  

};  // class ComputeRightEdgePathActionServer

}  // namespace garage_utils_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(garage_utils_pkg::ComputeRightEdgePathActionServer)


#endif