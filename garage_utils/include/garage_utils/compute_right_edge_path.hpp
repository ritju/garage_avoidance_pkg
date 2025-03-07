#ifndef GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_H_
#define GARAGE_UTILS__COMPUTE_RIGHT_EDGE_PATH_H_

#include <functional>
#include <memory>
#include <thread>

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

namespace garage_utils_pkg
{

// 顶点结构体（包含极角信息）
struct Vertex 
{
    double x, y;
    double angle; // 相对于矩形中心的极角
};

// 矩形结构体（增强版）
struct EnhancedRect 
{
    std::vector<Vertex> vertices;                             // 顶点集合（需排序后使用）
    std::vector<std::pair<size_t, size_t>> edges;             // 边集合（存储顶点索引）
    std::vector<size_t> adjacent_rects;                       // 相邻矩形索引列表
    size_t adjacent_rect_selected;                            // 最终选择的下个矩形索引值， -1代表最后一个
    bool visited = false;                                     // 遍历标记
    size_t index;                                             // 矩形索引值，用于调试矩形排序结果
    std::vector<std::pair<double, double>> middle_long_line;  // 矩形两条长边决定的遍历过程要覆盖的路径
};

class ComputeRightEdgePathActionServer : public rclcpp::Node
{
public:
  using ComputeRightEdgePath = garage_utils_msgs::action::ComputeRightEdgePath;
  using GoalHandleComputeRightEdgePath = rclcpp_action::ServerGoalHandle<ComputeRightEdgePath>;

  explicit ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ComputeRightEdgePathActionServer();

private:
  rclcpp_action::Server<ComputeRightEdgePath>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::vector<geometry_msgs::msg::Polygon> polygons_;
  std::vector<std::pair<bool, geometry_msgs::msg::Polygon>> polygons_pair_vec;

  size_t get_current_polygon_index(tf2::Transform robot_pose, std::vector<geometry_msgs::msg::Polygon> polygons);

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  tf2::Transform map_robot_tf;
  bool get_map_robot_tf();
  

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

// 极角排序函数（按顺时针排序顶点）
void sortVertices(EnhancedRect& rect) 
{
    // 计算矩形中心
    double cx = 0, cy = 0;
    for (const auto& v : rect.vertices) 
    {
        cx += v.x;
        cy += v.y;
    }
    cx /= rect.vertices.size();
    cy /= rect.vertices.size();

    // 计算各顶点极角并排序‌
    for (auto& v : rect.vertices) 
    {
        double dx = v.x - cx;
        double dy = v.y - cy;
        v.angle = atan2(dy, dx);  // 极角公式

    // 按极角升序排序（顺时针）
    std::sort(rect.vertices.begin(), rect.vertices.end(), 
        [](const Vertex& a, const Vertex& b) { return a.angle < b.angle; });
    }
}

// 生成矩形边（按排序后顶点顺序）
void generateEdges(EnhancedRect& rect) 
{
    rect.edges.clear();
    int n = rect.vertices.size();
    for (int i = 0; i < n; ++i) 
    {
        rect.edges.emplace_back(i, (i + 1) % n); // 边连接i和i+1顶点‌
    }
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

std::pair<double, double> generateDirection(std::pair<double, double> pre_path_end_point, std::vector<EnhancedRect>& rects)
{
    size_t rect_size = rects.size();
    if (rect_size < 2)
    {
        throw "rects's size shall >= 2 ";
    }

    auto rect_current = rects[rect_size - 1];
    double distance_min = std::numeric_limits<double>::max();
    size_t edge_start_index = -1;
    for (size_t i = 0; i < rect_current.edges.size(); i++)
    {
        auto edge = rect_current.edges[i];
        double middle_x = (rect_current.vertices[edge.first].x + rect_current.vertices[edge.second].x) / 2.0;
        double middle_y = (rect_current.vertices[edge.first].y + rect_current.vertices[edge.second].y) / 2.0;
        double distance_current = distance(middle_x, middle_y, pre_path_end_point.first, pre_path_end_point.second);
        if (distance_current < distance_min)
        {
            distance_min = distance_current;
            edge_start_index = i;
        }
    }
    auto edge_selected = rect_current.edges[edge_start_index];
    auto edge_next = rect_current.edges[(edge_start_index + 1) % (rect_current.edges.size())];
    double dx, dy;
    dx = rect_current.vertices[edge_next.first].x -rect_current.vertices[edge_selected.second].x;
    dy = rect_current.vertices[edge_next.first].y -rect_current.vertices[edge_selected.second].y;
    return {dx, dy};
}


// 递归遍历函数
void traverseRect(std::vector<EnhancedRect>& rects,  int current_idx,
                  std::vector<EnhancedRect>rects_ordered,
                  std::pair<double, double> dir, 
                  std::vector<std::pair<double, double>>& path) 
{
    if (rects[current_idx].visited) return;
    rects[current_idx].visited = true;
    rects_ordered.push_back(rects[current_idx]);

    // 选择右侧边并更新路径‌
    int edge_idx = selectRightEdge(rects[current_idx], dir);
    const auto& [start_idx, end_idx] = rects[current_idx].edges[edge_idx];
    path.emplace_back(rects[current_idx].vertices[start_idx].x, 
                     rects[current_idx].vertices[start_idx].y);
    path.emplace_back(rects[current_idx].vertices[end_idx].x, 
                     rects[current_idx].vertices[end_idx].y);

    // 计算新的行进方向
    double new_dx = rects[current_idx].vertices[end_idx].x - 
                   rects[current_idx].vertices[start_idx].x;
    double new_dy = rects[current_idx].vertices[end_idx].y - 
                   rects[current_idx].vertices[start_idx].y;

    // 递归遍历相邻矩形‌
    for (int adj : rects[current_idx].adjacent_rects) 
    {
        traverseRect(rects, adj, rects_ordered, { new_dx, new_dy },  path);
    }
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

  void generate_path_for_each_rect(EnhancedRect& rect, double px, double py, std::vector<std::pair<double, double>> &path)
  {
    rect.visited = true;
    std::pair<double, double> point_start, point_middle, point_end; // 遍历顺序  point_start => point_middle => point_end
    double seg1x, seg1y, seg2x, seg2y;
    seg1x = rect.middle_long_line[0].first;
    seg1y = rect.middle_long_line[0].second;
    seg2x = rect.middle_long_line[1].first;
    seg2y = rect.middle_long_line[1].second;
    point_start = find_neareast_point(px, py, seg1x, seg1y, seg2x, seg2y);
    if (rect.adjacent_rect_selected == -1)
    {
        double distance1, distance2;
        distance1 = distance(point_start.first, point_start.second, seg1x, seg1y);
        distance2 = distance(point_start.first, point_start.second, seg2x, seg2y);
        // 确定顺序
        if (distance1 < distance2)
        {
            point_middle = {seg1x, seg1y};
            point_end = {seg2x, seg2y};
        }
        else
        {
            point_middle = {seg2x, seg2y};
            point_end = {seg1x, seg1y};
        }

        // 第一段path
        std::pair<double, double> dir1 = {point_middle.first - point_start.first, point_middle.second - point_start.second};
        int edge1_index = selectRightEdge(rect, dir1);
        auto edge1 = rect.edges[edge1_index];
        auto path1_point_start = find_neareast_point(point_start.first, point_start.second, rect.vertices[edge1.first].x, rect.vertices[edge1.first].y, rect.vertices[edge1.second].x, rect.vertices[edge1.second].y);
        auto path1_point_end   = find_neareast_point(point_middle.first, point_middle.second, rect.vertices[edge1.first].x, rect.vertices[edge1.first].y, rect.vertices[edge1.second].x, rect.vertices[edge1.second].y);
        // 第二段path
        std::pair<double, double> dir2 = {point_end.first - point_middle.first, point_end.second - point_middle.second};
        int edge2_index = selectRightEdge(rect, dir2);
        auto edge2 = rect.edges[edge2_index];
        auto path2_point_start = find_neareast_point(point_middle.first, point_middle.second, rect.vertices[edge2.first].x, rect.vertices[edge2.first].y, rect.vertices[edge2.second].x, rect.vertices[edge2.second].y);
        auto path2_point_end   = find_neareast_point(point_end.first, point_end.second, rect.vertices[edge2.first].x, rect.vertices[edge2.first].y, rect.vertices[edge2.second].x, rect.vertices[edge2.second].y);        

        path.push_back(path1_point_start);
        path.push_back(path1_point_end);
        path.push_back(path2_point_start);
        path.push_back(path2_point_end);
    }
    else
    {

    }
    selectRightEdge(rect, );
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

  std::vector<EnhancedRect> rect_origin;
  std::vector<EnhancedRect> rect_ordered;

  bool isPointOnSegment(double px, double py, double x1, double y1, double x2, double y2);

  bool isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon);

  // 求解一个点到一条线段的距离：判断点到直线的垂线是否在线段上 1、在，返回点到直线的距离；2、不在，返回点到线段的两个端点的距离的平均值。
  double point_to_line_distance_smart(double point_x, double point_y, double line_pt1_x, double line_pt1_y, double line_pt2_x, double line_pt2_y);

};  // class ComputeRightEdgePathActionServer

}  // namespace garage_utils_pkg

RCLCPP_COMPONENTS_REGISTER_NODE(garage_utils_pkg::ComputeRightEdgePathActionServer)


#endif