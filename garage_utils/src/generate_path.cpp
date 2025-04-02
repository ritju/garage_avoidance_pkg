#include "garage_utils/generate_path.hpp"

namespace garage_utils_pkg
{
        GeneratePath::GeneratePath(rclcpp::Node::SharedPtr node)
        {
                this->node_ = node;
                RCLCPP_INFO(node_->get_logger(), "GeneratePath constructor."); 
                init_params();
        }

        GeneratePath::~GeneratePath()
        {
             RCLCPP_INFO(node_->get_logger(), "GeneratePath destructor.");   
        }

        void GeneratePath::process_(const std::vector<EnhancedPoint>& points, const std::vector<geometry_msgs::msg::Polygon>& polygons)
        {
              RCLCPP_INFO(node_->get_logger(), "Begin to generating path..."); 
              this->path_.header.frame_id = "map";
              this->path_.poses.clear();
               
               this->points_ = points;
               this->polygons_ = polygons;

        //        std::vector<nav_msgs::msg::Path> path_all;
               std::vector<std::pair<Point, Point>> path_all;
               nav_msgs::msg::Path path_one;

               for (size_t i = 0; i < points.size() - 1; i++)
               {
                    RCLCPP_INFO(node_->get_logger(), "");
                    RCLCPP_INFO(node_->get_logger(), "%zd/%zd", i, points.size()-2);
                    auto pt1 = points[i];
                    auto pt2 = points[i + 1];
                    int index = find_polygon_index(pt1, pt2, polygons);
                    auto path_of_one_segment = generate_path_for_one_segment2(pt1, pt2, polygons[index]);
                    
                    path_all.push_back(path_of_one_segment);
               }

               optimize_neighbored_path(path_all);
               this->path_.poses.clear();

               for (size_t i = 0; i < path_all.size(); i++)
               {
                        auto one_path = path_all[i];
                        auto path_start = one_path.first;
                        auto path_end = one_path.second;
                        // RCLCPP_INFO(node_->get_logger(), "path_end => (%f, %f)", path_end.first, path_end.second);

                        double dx, dy;
                        dx = path_end.first - path_start.first;
                        dy = path_end.second - path_start.second;
                        double length = std::sqrt(dx*dx + dy*dy);
                        double unit_x = dx / length;  
                        double unit_y = dy / length;   

                        nav_msgs::msg::Path path;
                        double distance_add = 0.0;
                        Point pt = path_start;

                        //       RCLCPP_INFO(node_->get_logger(), "***** path *****");
                        while (distance_add <= length)
                        {
                                geometry_msgs::msg::PoseStamped pose;
                                pose.header.frame_id = "map";
                                double x,y;
                                x = pt.first;
                                y = pt.second;
                                pose.pose.position.x = x;
                                pose.pose.position.y = y;
                                path.poses.push_back(pose);
                                // RCLCPP_INFO(node_->get_logger(), "pose: (%f, %f)", x, y);

                                pt.first +=  unit_x * this->resolution_;
                                pt.second += unit_y * this->resolution_; 
                                distance_add += this->resolution_;  
                                // RCLCPP_INFO(node_->get_logger(), "length: %f, distance_add: %f", length, distance_add);        
                        }
                        this->path_.poses.insert(this->path_.poses.end(), path.poses.begin(), path.poses.end());
               }
               
        //        for (size_t i = 0; i < path_all.size(); i++)
        //        {
        //             this->path_.poses.reserve(this->path_.poses.size() + path_all[i].poses.size());
        //             this->path_.poses.insert(this->path_.poses.end(), path_all[i].poses.begin(), path_all[i].poses.end());
        //        }
        }

        void GeneratePath::optimize_neighbored_path(std::vector<std::pair<Point, Point>>& path_all)
        {
                for (size_t i = 0; i < path_all.size() - 1; i++)
                {
                        auto &line1 = path_all[i];
                        auto &line2 = path_all[i+1];
                        update_two_neighbored_line(line1, line2);
                }
        }
        
        void GeneratePath::update_two_neighbored_line(std::pair<Point, Point>& line1, std::pair<Point, Point>& line2)
        {
                RCLCPP_INFO(node_->get_logger(), "line1: (%f, %f), (%f, %f)", 
                        line1.first.first, line1.first.second,
                        line1.second.first, line1.second.second);
                RCLCPP_INFO(node_->get_logger(), "line2: (%f, %f), (%f, %f)", 
                        line2.first.first, line2.first.second,
                        line2.second.first, line2.second.second);
                Point A = line1.first;
                Point B = line1.second;
                Point C = line2.first;
                Point D = line2.second;

                // 计算分母行列式
                double a = B.first - A.first;
                double b = -(D.first - C.first);
                double c = B.second - A.second;
                double d = -(D.second - C.second);

                double det = a * d - b * c;

                if (det == 0) 
                {
                        RCLCPP_INFO(node_->get_logger(), "parallel lines.");
                        // 平行线，无交点
                        return;
                }

                // 计算分子
                double D_t_x = C.first - A.first;
                double D_t_y = C.second - A.second;
                double D_t = D_t_x * d - D_t_y * b;
                double t = D_t / det;

                double D_s = a * D_t_y - c * D_t_x;
                double s = D_s / det;

                // 计算交点坐标
                double inter_x = A.first + t * (B.first - A.first);
                double inter_y = A.second + t * (B.second - A.second);

                Point inter = {inter_x, inter_y};

                // 判断交点是否在线段上
                bool on_line1 = (t >= 0.0 && t <= 1.0);
                bool on_line2 = (s >= 0.0 && s <= 1.0);

                RCLCPP_INFO(node_->get_logger(), "t: %f, s: %f", t, s);

                if (on_line1) 
                {

                        // 处理线段1的更新
                        Point& p1 = line1.first;
                        Point& p2 = line1.second;

                        // 计算平方距离
                        double dist_p1_sq = (inter.first - p1.first) * (inter.first - p1.first) + (inter.second - p1.second) * (inter.second - p1.second);
                        double dist_p2_sq = (inter.first - p2.first) * (inter.first - p2.first) + (inter.second - p2.second) * (inter.second - p2.second);

                        if (dist_p1_sq <= dist_p2_sq) 
                        {
                                RCLCPP_INFO(node_->get_logger(), "update p1 with (%f, %f)", inter.first, inter.second);
                                p1 = inter;
                        }
                        else 
                        {
                                RCLCPP_INFO(node_->get_logger(), "update p2 with (%f, %f)", inter.first, inter.second);
                                p2 = inter;
                                RCLCPP_INFO(node_->get_logger(), "new    p2   is (%f, %f)", line1.second.first, line1.second.second);
                        }

                } 
                else if (on_line2) 
                {
                        // 处理线段2的更新
                        Point& p3 = line2.first;
                        Point& p4 = line2.second;
                        double dist_p3_sq = (inter.first - p3.first) * (inter.first - p3.first) + (inter.second - p3.second) * (inter.second - p3.second);
                        double dist_p4_sq = (inter.first - p4.first) * (inter.first - p4.first) + (inter.second - p4.second) * (inter.second - p4.second);

                        if (dist_p3_sq <= dist_p4_sq) 
                        {
                                RCLCPP_INFO(node_->get_logger(), "update p3 with (%f, %f)", inter.first, inter.second);
                                p3 = inter;
                        } 
                        else 
                        {
                                RCLCPP_INFO(node_->get_logger(), "update p4 with (%f, %f)", inter.first, inter.second);
                                p4 = inter;
                        }
                }
        }

        int GeneratePath::find_polygon_index(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const std::vector<geometry_msgs::msg::Polygon>& polygons)
        {
                RCLCPP_INFO(node_->get_logger(), "Begin to find rect index"); 
                RCLCPP_INFO(node_->get_logger(), "pt1 => (%f, %f)", pt1.coord.first, pt1.coord.second);
                RCLCPP_INFO(node_->get_logger(), "pt2 => (%f, %f)", pt2.coord.first, pt2.coord.second);
                double px, py;
                px = (pt1.coord.first + pt2.coord.first) / 2.0;
                py = (pt1.coord.second + pt2.coord.second) / 2.0;

                for (size_t i = 0; i < polygons.size(); i++)
                {
                        std::vector<std::pair<double, double>> polygon_compare;

                        geometry_msgs::msg::Polygon polygon_one = polygons[i];
                        
                        for (size_t j = 0; j < polygon_one.points.size(); j++)
                        {
                                std::pair<double, double> pt;
                                pt.first = polygon_one.points[j].x;
                                pt.second = polygon_one.points[j].y;
                                polygon_compare.push_back(pt);
                        }
                        if (isPointInPolygon(px, py, polygon_compare))
                        {
                                RCLCPP_INFO(node_->get_logger(), "result => rect index: %zd, [ (%f, %f), (%f, %f), (%f, %f), (%f, %f) ]", 
                                        i,
                                        polygon_compare[0].first, polygon_compare[0].second,
                                        polygon_compare[1].first, polygon_compare[1].second,
                                        polygon_compare[2].first, polygon_compare[2].second,
                                        polygon_compare[3].first, polygon_compare[3].second
                                        );
                                return i;
                        }
                }
                 

                std::stringstream ss;
                ss << "can not find pt1(" << pt1.coord.first << ", " << pt1.coord.second << ")"
                        << "and pt2("     << pt2.coord.first << ", " << pt2.coord.second << ")"
                        << " in polygons";
                throw ss.str().c_str();
        }

        void GeneratePath::sortVertices(EnhancedRect& rect)
        {
                // 计算矩形中心
                double cx = 0, cy = 0;
                for (const auto& v : rect.vertices) 
                {
                        cx += v.x;
                        cy += v.y;
                        // RCLCPP_INFO(node_->get_logger(), "x: %.1f, y: %.1f, cx: %.1f, cy: %.1f", v.x, v.y, cx, cy);
                }
                cx /= rect.vertices.size();
                cy /= rect.vertices.size();
                // RCLCPP_INFO(node_->get_logger(), "cx: %.1f, cy: %.1f", cx, cy);

                // 计算各顶点极角并排序‌
                for (auto& v : rect.vertices) 
                {
                        double dx = v.x - cx;
                        double dy = v.y - cy;
                        v.angle = atan2(dy, dx);  // 极角公式
                        // RCLCPP_INFO(node_->get_logger(), "x: %.1f, y: %.1f, cx: %.1f, cy: %.1f", v.x, v.y, cx, cy);
                        // RCLCPP_INFO(node_->get_logger(), "dx: %.1f, dy: %.1f, v.angle: %.2f", dx, dy, v.angle);

                }
                // 按极角升序排序（顺时针）
                std::sort(rect.vertices.begin(), rect.vertices.end(), 
                        [](const Vertex& a, const Vertex& b) { return a.angle < b.angle; });
        }

        nav_msgs::msg::Path GeneratePath::generate_path_for_one_segment(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const geometry_msgs::msg::Polygon& polygon)
        {
              RCLCPP_INFO(node_->get_logger(), "Begin to generating path for one segment."); 
              EnhancedRect rect;
              convert_polygon_to_rect(polygon, rect);
              auto edge = find_edge(pt1, pt2, rect);
              std::pair<double, double> edge_start, edge_end, path_start, path_end;
              
              edge_start = find_neareast_point(pt1.coord.first, pt1.coord.second, edge.first.first, edge.first.second, edge.second.first, edge.second.second);
              edge_end =   find_neareast_point(pt2.coord.first, pt2.coord.second, edge.first.first, edge.first.second, edge.second.first, edge.second.second);

              auto offset_line = offset(this->offset_, edge_start, edge_end);
              path_start = offset_line[0];
              path_end = offset_line[1];
              RCLCPP_INFO(node_->get_logger(), "path_start: (%f, %f)", path_start.first, path_start.second);
              RCLCPP_INFO(node_->get_logger(), "path_end  : (%f, %f)", path_end.first, path_end.second);

              double dx, dy;
              dx = path_end.first - path_start.first;
              dy = path_end.second - path_start.second;
              double length = std::sqrt(dx*dx + dy*dy);
              double unit_x = dx / length;  
              double unit_y = dy / length;   

              nav_msgs::msg::Path path;
              double distance_add = 0.0;
              Point pt = path_start;

        //       RCLCPP_INFO(node_->get_logger(), "***** path *****");
              while (distance_add <= length)
              {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                double x,y;
                x = pt.first;
                y = pt.second;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                path.poses.push_back(pose);
                // RCLCPP_INFO(node_->get_logger(), "pose: (%f, %f)", x, y);

                pt.first +=  unit_x * this->resolution_;
                pt.second += unit_y * this->resolution_; 
                distance_add += this->resolution_;  
                // RCLCPP_INFO(node_->get_logger(), "length: %f, distance_add: %f", length, distance_add);        
              }
              return path;
        }

        std::pair<Point, Point> GeneratePath::generate_path_for_one_segment2(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const geometry_msgs::msg::Polygon& polygon)
        {
              RCLCPP_INFO(node_->get_logger(), "Begin to generating path for one segment."); 
              EnhancedRect rect;
              convert_polygon_to_rect(polygon, rect);
              auto edge = find_edge(pt1, pt2, rect);
              std::pair<double, double> edge_start, edge_end, path_start, path_end;
              
              edge_start = find_neareast_point(pt1.coord.first, pt1.coord.second, edge.first.first, edge.first.second, edge.second.first, edge.second.second);
              edge_end =   find_neareast_point(pt2.coord.first, pt2.coord.second, edge.first.first, edge.first.second, edge.second.first, edge.second.second);

              auto offset_line = offset(this->offset_, edge_start, edge_end);
              path_start = offset_line[0];
              path_end = offset_line[1];
              RCLCPP_INFO(node_->get_logger(), "path_start: (%f, %f)", path_start.first, path_start.second);
              RCLCPP_INFO(node_->get_logger(), "path_end  : (%f, %f)", path_end.first, path_end.second);

              return {path_start, path_end};
        }

        void GeneratePath::convert_polygon_to_rect(const geometry_msgs::msg::Polygon& polygon, EnhancedRect& rect)
        {
                rect.vertices.clear();
                for (size_t i = 0; i < polygon.points.size(); i++)
                {
                        Vertex v;
                        v.x = polygon.points[i].x;
                        v.y = polygon.points[i].y;
                        rect.vertices.push_back(v);
                }
                sortVertices(rect);
        }

        void GeneratePath::init_params()
        {
                this->offset_ = this->node_->get_parameter_or<double>("offset", 0.2);
                this->resolution_ = this->node_->get_parameter_or<double>("resolution", 2.5);
        }

        std::vector<std::pair<double, double>> GeneratePath::offset(double dis, std::pair<double, double> pt1, std::pair<double, double> pt2) 
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

        std::pair<Point, Point> GeneratePath::find_edge(const EnhancedPoint& pt1, const EnhancedPoint& pt2, const EnhancedRect& rect)
        {
                RCLCPP_INFO(node_->get_logger(), "Begin to find edge.");
                RCLCPP_INFO(node_->get_logger(), "pt1 => (%f, %f)", pt1.coord.first, pt1.coord.second);
                RCLCPP_INFO(node_->get_logger(), "pt2 => (%f, %f)", pt2.coord.first, pt2.coord.second);
                RCLCPP_INFO(node_->get_logger(), "rect => [ [%f, %f), (%f, %f), (%f, %f), (%f, %f)", 
                        rect.vertices[0].x, rect.vertices[0].y,
                        rect.vertices[1].x, rect.vertices[1].y,
                        rect.vertices[2].x, rect.vertices[2].y,
                        rect.vertices[3].x, rect.vertices[3].y
                        );

                std::pair<Point, Point> edge;
                double dx1, dy1, dx2, dy2;
                
                dx1 = pt2.coord.first - pt1.coord.first;
                dy1 = pt2.coord.second - pt1.coord.second;

                double dot_max = std::numeric_limits<double>::min();
                size_t index_start, index_end;

                for (size_t i = 0; i < rect.vertices.size(); i++)
                {
                        // RCLCPP_INFO(node_->get_logger(), "vertices %zd", i);
                        auto pt1 = rect.vertices[i];
                        auto pt2 = rect.vertices[(i+1)%4];
                        dx2 = pt2.x - pt1.x;
                        dy2 = pt2.y - pt1.y;
                        double dot = dx1 * dx2 + dy1 * dy2;
                        if (dot > dot_max)
                        {
                                dot_max = dot;
                                index_start = i;
                        }
                }
                index_end = (index_start + 1) % rect.vertices.size();

                edge.first.first = rect.vertices[index_start].x;
                edge.first.second = rect.vertices[index_start].y;
                edge.second.first = rect.vertices[index_end].x;
                edge.second.second = rect.vertices[index_end].y;

                RCLCPP_INFO(node_->get_logger(), "result edge => (%f, %f), (%f, %f)", 
                        edge.first.first, edge.first.second,
                        edge.second.first, edge.second.second);

                return edge;
        }

        bool GeneratePath::isPointOnSegment(double px, double py, double x1, double y1, double x2, double y2)
        {
                // 判断点是否在线段端点范围内
                if ((px < std::min(x1, x2) - 1e-8) || (px > std::max(x1, x2) + 1e-8) ||
                        (py < std::min(y1, y2) - 1e-8) || (py > std::max(y1, y2) + 1e-8)) 
                        return false;

                // 共线检测‌
                double cross = (x2 - x1)*(py - y1) - (y2 - y1)*(px - x1);
                return fabs(cross) < 1e-8;
        }

        bool GeneratePath::isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon)
        {
                if (polygon.size() < 3) return false;

                bool result = false;
                const double infinity = std::numeric_limits<double>::max();
                int n = polygon.size();

                for (int i = 0; i < n; ++i) 
                {
                        const auto& p1 = polygon[i];
                        const auto& p2 = polygon[(i+1)%n];

                        // 首先检查点是否在边上
                        if (isPointOnSegment(px, py, p1.first, p1.second, p2.first, p2.second))
                        return true;

                        // 射线与边段的相交判断‌
                        if ((p1.second > py) != (p2.second > py)) 
                        {
                                // 计算射线与边的交点x坐标‌:ml-citation{ref="1,2" data="citationList"}
                                double intersectX = (p2.first - p1.first) * (py - p1.second) 
                                                / (p2.second - p1.second) + p1.first;

                                // 处理水平边的情况‌
                                if (p1.second == p2.second) continue;

                                // 判断交点是否在射线右侧
                                if (px <= intersectX) 
                                result = !result;
                        }
                }
                return result;
        }

         // 找到某一个点在线段上离它最近的点  
        std::pair<double, double> GeneratePath::find_neareast_point(double px, double py, double seg1x, double seg1y, double seg2x, double seg2y)
        {
                RCLCPP_INFO(node_->get_logger(), "find_neareast_point");
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

                RCLCPP_INFO(node_->get_logger(), "result => x: %f, y: %f", nearest_x, nearest_y);
                return {nearest_x, nearest_y}; // 返回pair结构‌:
        }
        
} // end of namespace



