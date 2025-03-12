#include "garage_utils/generate_model.hpp"
#include <sstream>

namespace garage_utils_pkg
{
        GenerateModel::GenerateModel(std::vector<std::vector<Point>> rects, rclcpp::Node::SharedPtr node, double dis_thr)
        {
                this->rects_ = rects;
                this->node_ = node;
                this->dis_thr_ = dis_thr;
                RCLCPP_INFO(node_->get_logger(), "GenerateModel constructor.");
                check_rects(rects_);
                print_rects(rects_);
                rects_tmp_ = rectify_rects(rects_);
                // generate_model(rects_tmp_, this->dis_thr_); 
        }

        GenerateModel::~GenerateModel()
        {

        }

        void GenerateModel::assert_(bool condition, std::string error_msg)
        {
                if (!condition)
                {
                        throw error_msg;
                }
        }

        void GenerateModel::check_rects(std::vector<std::vector<Point>> rects)
        {
                assert_(rects.size() > 0, "rects's size shall > 0, error");
                for (size_t i = 0; i < rects.size(); i++)
                {
                        auto rect = rects[i];
                        assert_(rect.size() == 4, "rect shall has 4 points.");
                        for (size_t j = 0; j < rect.size() - 1; j++)
                        {
                                for (size_t k = j+1; k < rect.size(); k++)
                                {
                                        assert_(rect[j] != rect[k], "the 4 points of rect shall be different.");
                                }
                        }
                }
        }

        void GenerateModel::print_rects(std::vector<std::vector<Point>> rects)
        {
                size_t size = rects.size();
                RCLCPP_INFO(node_->get_logger(), "rests size: %zd", size);
                std::vector<std::stringstream> rects_ss;
                for (size_t i = 0; i < size; i++)
                {
                        auto rect = rects[i];
                        std::stringstream ss;
                        ss << "rect " << i << " => [";
                        for (size_t j = 0; j < rect.size(); j++)
                        {
                                ss << " (" << rect[j].first << ", " << rect[j].second << ")";
                        }
                        if (i != size - 1)
                        {
                                ss << ",";
                        }
                        else
                        {
                                ss<< " ";
                        }
                        ss<< "]";
                        RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
                }
        }

        std::vector<EnhancedRect> GenerateModel::rectify_rects(const std::vector<std::vector<Point>>& rects)
        {
                return init_rects(rects);
        }

        std::vector<EnhancedRect> GenerateModel::init_rects(std::vector<std::vector<Point>> rects)
        {
                std::vector<EnhancedRect> rects_ret;
                for (size_t i = 0; i < rects.size(); i++)
                {
                        auto rect_origin = rects[i];
                        EnhancedRect enhanced_rect;
                        for (size_t j = 0; j < rect_origin.size(); j++)
                        {
                                Vertex vertex;
                                vertex.x = rect_origin[j].first;
                                vertex.y = rect_origin[j].second;
                                enhanced_rect.vertices.push_back(vertex);
                        }
                        sortVertices(enhanced_rect);
                        generate_middle_long_line(enhanced_rect);
                        rects_ret.push_back(enhanced_rect);

                }
                return rects_ret;
        }

        void GenerateModel::sortVertices(EnhancedRect& rect)
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

        void GenerateModel::generate_middle_long_line(EnhancedRect& rect)
        {
                auto vertices = rect.vertices;
                double distance_max = std::numeric_limits<double>::min();
                int index_start, index_end;
                for (size_t i = 0; i < vertices.size(); i++)
                {
                        Point pt1, pt2;
                        pt1.first = vertices[i].x;
                        pt1.second = vertices[i].y;
                        pt2.first = vertices[(i+1)%4].x;
                        pt2.second = vertices[(i+1)%4].y;
                        double distance_current = std::hypot(pt1.first - pt2.first, pt1.second - pt2.second);
                        if (distance_current > distance_max)
                        {
                                distance_max = distance_current;
                                index_start = i;
                                index_end = (i + 1) % 4;
                        }
                }

                Point pt0, pt1, pt2, pt3; // 假定 pt3->pt0 和 pt1->pt2为短边， pt0->pt1 和 pt2->pt3为长边，
                pt0.first = rect.vertices[index_start].x;
                pt0.second = rect.vertices[index_start].y;
                pt1.first = rect.vertices[index_end].x;
                pt1.second = rect.vertices[index_end].y;
                pt2.first = rect.vertices[(index_end + 1) % 4].x;
                pt2.second = rect.vertices[(index_end + 1) % 4].y;
                pt3.first = rect.vertices[(index_end + 2) % 4].x;
                pt3.second = rect.vertices[(index_end + 2) % 4].y;

                Point long_line_pt1, long_line_pt2;
                long_line_pt1.first = (pt0.first + pt3.first) / 2.0;
                long_line_pt1.second = (pt0.second + pt3.second) / 2.0;
                long_line_pt2.first = (pt1.first + pt2.first) / 2.0;
                long_line_pt2.second = (pt1.second + pt2.second) / 2.0;
                
                rect.middle_long_line.push_back(long_line_pt1);
                rect.middle_long_line.push_back(long_line_pt2);
        }
        
        std::vector<Edge> GenerateModel::generate_edges(std::vector<EnhancedRect> rects)
        {
                std::vector<Edge> edges;
                for (size_t i = 0; i < rects.size(); i++)
                {
                        Edge edge;
                        edge.pt1 = rects[i].middle_long_line[0];
                        edge.pt2 = rects[i].middle_long_line[1];
                        edges.push_back(edge);
                }
                return edges;
        }

        void GenerateModel::generate_model(const std::vector<EnhancedRect>& rects, double dis_thr)
        {
                auto edges = generate_edges(rects);    
                size_t edges_size = edges.size();
                assert_(edges_size > 0, "edges's size shall > 0");

                // 添加相邻关系信息
                for (size_t i = 0; i < edges_size - 1; i++)
                {
                        for (size_t j = i + 1; j < edges_size; j++)
                        {
                                double edges_distance = compute_distance_of_two_edges(edges[i].pt1, edges[i].pt2, edges[j].pt1, edges[j].pt2);
                                if (edges_distance < dis_thr)
                                {
                                        edges[i].adjacent_vec.push_back(j);
                                        edges[j].adjacent_vec.push_back(i);
                                }
                                else
                                {
                                        continue;
                                }
                        }
                }

                // 初始化points;
                points_.clear();
                int index = 0;
                for (size_t i = 0; i < edges.size(); i++)
                {
                        EnhancedPoint e_pt1, e_pt2;
                        e_pt1.visited = false;
                        e_pt2.visited = false;
                        e_pt1.index = index++;
                        e_pt2.index = index++;
                        add_adjacent_relation(e_pt1, e_pt2);
                        edges[i].pt1_index = e_pt1.index;
                        edges[i].pt2_index = e_pt2.index;
                }

                for (size_t i = 0; i < edges.size(); i++)
                {
                        auto edge = edges[i];
                        if (edge.adjacent_vec.size() > 0)
                        {
                                for (size_t j = 0; j < edge.adjacent_vec.size() && !edge.deleted; j++)
                                {
                                        auto edge_adjacent = edges[edge.adjacent_vec[j]];
                                        process_intersected_edges(edge, edge_adjacent);
                                }
                        }
                        else
                        {
                                RCLCPP_WARN(node_->get_logger(), "edge %zd has 0 adjacent edge.[(%f, %f), (%f, %f)]",
                                        i, edge.pt1.first, edge.pt1.second, edge.pt2.first, edge.pt2.second );
                        }
                }

        }

        void GenerateModel::add_adjacent_relation(EnhancedPoint& pt1, EnhancedPoint& pt2)
        {
                if (std::find(pt1.adjacent_vec.begin(), pt1.adjacent_vec.end(), pt2.index) == pt1.adjacent_vec.end())
                {
                        pt1.adjacent_vec.push_back(pt2.index);
                }
                if (std::find(pt2.adjacent_vec.begin(), pt2.adjacent_vec.end(), pt1.index) == pt2.adjacent_vec.end())
                {
                        pt2.adjacent_vec.push_back(pt1.index);
                }
        }

        void GenerateModel::remove_adjacent_relation(EnhancedPoint& pt1, EnhancedPoint& pt2)
        {
                auto iter1 = std::find(pt1.adjacent_vec.begin(), pt1.adjacent_vec.end(), pt2.index);
                if (iter1 != pt1.adjacent_vec.end())
                {
                        pt1.adjacent_vec.erase(iter1);
                }

                auto iter2 = std::find(pt2.adjacent_vec.begin(), pt2.adjacent_vec.end(), pt1.index);
                if (iter2 != pt2.adjacent_vec.end())
                {
                        pt2.adjacent_vec.erase(iter2);
                }
        }

        double GenerateModel::compute_distance_of_two_edges(Point edge1_pt1, Point edge1_pt2, Point edge2_pt1, Point edge2_pt2)
        {
                double cx1,cy1, cx2,cy2;
                cx1 = (edge1_pt1.first + edge1_pt2.first) / 2.0;
                cy1 = (edge1_pt1.second + edge1_pt2.second) / 2.0;
                cx2 = (edge2_pt1.first + edge2_pt2.first) / 2.0;
                cy2 = (edge2_pt1.second + edge2_pt2.second) / 2.0;
                double distance1 = point_to_line_distance_smart(cx1, cy1, edge2_pt1.first, edge2_pt1.second, edge2_pt2.first, edge2_pt2.second);
                double distance2 = point_to_line_distance_smart(cx2, cy2, edge1_pt1.first, edge1_pt1.second, edge1_pt2.first, edge1_pt2.second);
                return (distance1 < distance2 ? distance1 : distance2);
        }

        double GenerateModel::point_to_line_distance_smart(double px, double py, double x1, double y1, double x2, double y2)
        {                
                double fx, fy; // 点到线段的垂线与线段所在的直线的交点
                double t;      // fy = y1 + t * (y2 - y1); fx = x1 + t * (x2 - x1)
                double dx = x2 - x1;
                double dy = y2 - y1;
                // 线段的两个端点为同一个点
                if (x1 == x2 && y1 == y2)
                {
                        fx = x1;
                        fy = y1;
                        return distance(px, py, fx, fy);
                }
                else
                {
                        double vx = px - x1;
                        double vy = py - y1;
                        t = (vx * dx + vy * dy) / (dx * dx + dy * dy);
                        fx = x1 + t * (x2 - x1);
                        fy = y1 + t * (y2 - y1);
                }
                
                if (t > 0 && t <= 1.0)
                {
                        return distance(px, py, fx, fy);
                }
                else
                {
                        return (distance(px, py, x1, y1) + distance(px, py, x2, y2)) / 2.0;
                }
        }

        void GenerateModel::process_intersected_edges(Edge& edge1, Edge& edge2)
        {
                
        }

        

} // end of namespace


