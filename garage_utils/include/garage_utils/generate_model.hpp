#ifndef GARAGE_UTILS__GENERATE_MODEL_HPP_
#define GARAGE_UTILS__GENERATE_MODEL_HPP_

#include "garage_utils/types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace garage_utils_pkg
{

class GenerateModel
{
public:
        GenerateModel(const std::vector<std::vector<Point>>& rects, rclcpp::Node::SharedPtr node, double dis_thr);
        ~GenerateModel();

        void check_rects(std::vector<std::vector<Point>> rects);
        void assert_(bool condition, std::string errors);
        void print_rects(const std::vector<std::vector<Point>>& rects);
        void print_enhanced_rects(const std::vector<EnhancedRect>& rects);
        std::vector<EnhancedRect> init_rects(const std::vector<std::vector<Point>>& rects);
        void sortVertices(EnhancedRect& rect);
        void generate_middle_long_line(EnhancedRect& rect);
        std::vector<Edge> generate_edges(std::vector<EnhancedRect> rects);

        void generate_model(const std::vector<EnhancedRect>& rects, double dis_thr);
        std::vector<EnhancedPoint> get_model();

        // 计算两条边的距离
        double compute_distance_of_two_edges(Point edge1_pt1, Point edge1_pt2, Point edge2_pt1, Point edge2_pt2);
        // 求解一个点到一条线段的距离：判断点到直线的垂线是否在线段上 1、在，返回点到直线的距离；2、不在，返回点到线段的两个端点的距离的平均值。
        double point_to_line_distance_smart(double point_x, double point_y, double line_pt1_x, double line_pt1_y, double line_pt2_x, double line_pt2_y);
        double distance(double x1, double y1, double x2, double y2)
        {     
                return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
        }

        // 添加两个点的相邻关系
        void add_adjacent_relation(EnhancedPoint& pt1, EnhancedPoint& pt2);
        // 删除两个点的相邻关系
        void remove_adjacent_relation(EnhancedPoint& pt1, EnhancedPoint& pt2);

        // 找到某一个点在线段上离它最近的点  
        Point find_neareast_point(double px, double py, double seg1x, double seg1y, double seg2x, double seg2y);

        // 判断两个edge是否相邻
        bool is_neighbor(const Edge& e1, const Edge& e2, double dis_thr);

        // 处理相交的边
        void process_intersected_edges(Edge& edge1, Edge& edge2, std::vector<Edge>& edges, std::vector<EnhancedPoint>& points);

        void merge_edges(std::vector<Edge> edges_origin, int index, std::vector<Edge> edges_tmp);

        void generate_points(std::vector<EnhancedPoint>& points, const std::vector<Edge>& edges, double dis_thr);

        std::vector<EnhancedPoint> get_points()
        {
                return points_;
        }

private:
        double dis_thr_;
        std::vector<std::vector<Point>>    rects_;     // 输入
        std::vector<EnhancedRect>          rects_tmp_;
        std::vector<EnhancedPoint>         points_;    // 输出
        rclcpp::Node::SharedPtr            node_;
        int                                index;      // 生成points时的索引值。
        
};

} // end of namespace

#endif