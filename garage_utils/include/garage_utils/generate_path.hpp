#ifndef GARAGE_UTILS__GENERATE_PATH_HPP_
#define GARAGE_UTILS__GENERATE_PATH_HPP_

#include "garage_utils/types.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon.hpp"


namespace garage_utils_pkg
{
        
        class GeneratePath
        {
        public:
                explicit GeneratePath(rclcpp::Node::SharedPtr node);
                ~GeneratePath();

                void process_(const std::vector<EnhancedPoint>& points, const std::vector<geometry_msgs::msg::Polygon>& polygons);
                int find_polygon_index(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const std::vector<geometry_msgs::msg::Polygon>& polygons);
                std::pair<Point, Point> find_edge(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const EnhancedRect& rect);
                nav_msgs::msg::Path generate_path_for_one_segment(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const geometry_msgs::msg::Polygon& polygons);
                std::pair<Point, Point> generate_path_for_one_segment2(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const geometry_msgs::msg::Polygon& polygons);

                bool isPointOnSegment(double px, double py, double x1, double y1, double x2, double y2);
                bool isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon);
                void sortVertices(EnhancedRect& rect); 
                // 找到某一个点在线段上离它最近的点  
                std::pair<double, double> find_neareast_point(double px, double py, double seg1x, double seg1y, double seg2x, double seg2y);  

                nav_msgs::msg::Path get_path()
                {
                        return path_;
                }

                void optimize_neighbored_path(std::vector<std::pair<Point, Point>>& path_all);
                void update_two_neighbored_line(std::pair<Point, Point>& line1, std::pair<Point, Point>& line2);

                void convert_polygon_to_rect(const geometry_msgs::msg::Polygon& polygon, EnhancedRect& rect);

                void init_params();
                std::vector<std::pair<double, double>> offset(double dis, std::pair<double, double> pt1, std::pair<double, double> pt2);

                // 递归函数
                
        
        private:
                rclcpp::Node::SharedPtr node_;
                nav_msgs::msg::Path path_;
                std::vector<EnhancedPoint> points_;
                std::vector<geometry_msgs::msg::Polygon> polygons_;

                // params
                double offset_;
                double resolution_;
        };
} // end of namespace

#endif





