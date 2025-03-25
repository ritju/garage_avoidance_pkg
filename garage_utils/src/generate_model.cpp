#include "garage_utils/generate_model.hpp"
#include <sstream>

namespace garage_utils_pkg
{
        GenerateModel::GenerateModel(rclcpp::Node::SharedPtr node)
        {
                this->node_ = node;
                RCLCPP_INFO(node_->get_logger(), "GenerateModel constructor.");
        }

        GenerateModel::~GenerateModel()
        {
                RCLCPP_INFO(node_->get_logger(), "GenerateModel destructor.");
        }

        void GenerateModel::process_(const std::vector<std::vector<Point>>& rects, double dis_thr)
        {
                this->rects_ = rects;
                this->dis_thr_ = dis_thr;
                index = 0;
                RCLCPP_INFO(node_->get_logger(), "Check rects.");

                check_rects(rects_);

                RCLCPP_INFO(node_->get_logger(), "Print rects");
                print_rects(rects_);

                rects_tmp_ = init_rects(rects_);  

                RCLCPP_INFO(node_->get_logger(), "Begin to generate model.");
                generate_model(rects_tmp_, this->dis_thr_); 
        }      

        void GenerateModel::check_rects(std::vector<std::vector<Point>> rects)
        {
                RCLCPP_INFO(node_->get_logger(), "rects size: %zd", rects.size());
                assert_(rects.size() > 0, "rects's size shall > 0, error");
                for (size_t i = 0; i < rects.size(); i++)
                {
                        auto rect = rects[i];
                        RCLCPP_INFO(node_->get_logger(), "rect points size: %zd", rect.size());
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

        void GenerateModel::print_rects(const std::vector<std::vector<Point>>& rects)
        {
                size_t size = rects.size();
                RCLCPP_INFO(node_->get_logger(), "original rects size: %zd", size);
                std::vector<std::stringstream> rects_ss;
                for (size_t i = 0; i < size; i++)
                {
                        auto rect = rects[i];
                        std::stringstream ss;
                        ss << "rect " << i << " => [ ";
                        for (size_t j = 0; j < rect.size(); j++)
                        {
                                ss << "(" << rect[j].first << ", " << rect[j].second << ")";
                                if (j != rect.size() - 1)
                                {
                                        ss << ", ";
                                }
                                else
                                {
                                        ss << " ";
                                }
                        }
                        ss<< "]";
                        RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
                }
        }

        void GenerateModel::print_enhanced_rects(const std::vector<EnhancedRect>& rects)
        {
                size_t size = rects.size();
                RCLCPP_INFO(node_->get_logger(), "Enhanced rects size: %zd", size);
                std::vector<std::stringstream> rects_ss;
                for (size_t i = 0; i < size; i++)
                {
                        auto rect = rects[i];
                        std::stringstream ss;
                        ss << "rect " << i << " => [ ";
                        for (size_t j = 0; j < rect.vertices.size(); j++)
                        {
                                ss << "(" << rect.vertices[j].x << ", " << rect.vertices[j].y << ", " << rect.vertices[j].angle << ")";
                                if (j != rect.vertices.size() - 1)
                                {
                                        ss << ", ";
                                }
                                else
                                {
                                        ss << " ";
                                }
                        }
                        ss<< "]";
                        RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
                        RCLCPP_INFO(node_->get_logger(), "middle_long_line: (%.1f, %.1f), (%.1f, %.1f)",
                                rect.middle_long_line[0].first, rect.middle_long_line[0].second, 
                                rect.middle_long_line[1].first, rect.middle_long_line[1].second);
                        RCLCPP_INFO(node_->get_logger(), "--------------------------");
                }
        }

        void GenerateModel::print_edges(const std::vector<Edge> edges)
        {
                RCLCPP_INFO(node_->get_logger(), "===== print edges =====");
                assert_(edges.size() > 0, "edges size shall > 0");
                for (size_t i = 0; i < edges.size(); i++)
                {
                        auto edge = edges[i];
                        RCLCPP_INFO(node_->get_logger(), "edge %zd", i);
                        RCLCPP_INFO(node_->get_logger(), "pt1 => index: %d, coord: (%.1f, %.1f)", edge.pt1_index, edge.pt1.first, edge.pt1.second);
                        RCLCPP_INFO(node_->get_logger(), "pt2 => index: %d, coord: (%.1f, %.1f)", edge.pt2_index, edge.pt2.first, edge.pt2.second);
                }
        }

        std::vector<EnhancedRect> GenerateModel::init_rects(const std::vector<std::vector<Point>>& rects)
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
                print_enhanced_rects(rects_ret);
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
                        edge.pt1_index = index++;
                        edge.pt2_index = index++;
                        edge.dis = distance(edge.pt1.first, edge.pt1.second, edge.pt2.first, edge.pt2.second);
                        edges.push_back(edge);
                }
                return edges;
        }

        void GenerateModel::generate_model(const std::vector<EnhancedRect>& rects, double dis_thr)
        {
                index = 0;
                auto edges = generate_edges(rects);
                sort_edges(edges);
                print_edges(edges);  
                size_t edges_size = edges.size();
                assert_(edges_size > 0, "edges's size shall > 0");

                // 先清空points_
                points_.clear();
                generate_points(points_, edges, dis_thr);
        }

        bool GenerateModel::is_neighbor(const Edge& e1, const Edge& e2, double dis_thr)
        {
                double edges_distance = compute_distance_of_two_edges(e1, e2);
                RCLCPP_INFO(node_->get_logger(), "edges_distance: %f, dis_thr: %f", edges_distance, dis_thr);
                return edges_distance < dis_thr;
        }

        void GenerateModel::generate_points(std::vector<EnhancedPoint>& points, const std::vector<Edge>& edges, double dis_thr)
        {
                RCLCPP_INFO(node_->get_logger(), "generate_points");
                std::vector<Edge> edges_added; // 用来存放已经添加过的edges

                for (size_t i = 0; i < edges.size(); i++)
                {
                        RCLCPP_INFO(node_->get_logger(), "");
                        RCLCPP_INFO(node_->get_logger(), "========== loop %zd ==========", i);
                        auto edge = edges[i];
                        if (edges_added.size() == 0) // 第一次遍历时
                        {                           
                           
                           RCLCPP_INFO(node_->get_logger(), "first loop, init points");
                           EnhancedPoint e_pt1, e_pt2;
                           
                           e_pt1.index = edge.pt1_index;
                           e_pt1.visited = false;
                           e_pt1.coord = edge.pt1;

                           e_pt2.index = edge.pt2_index;
                           e_pt2.visited = false;
                           e_pt2.coord = edge.pt2;
                           
                           add_adjacent_relation(e_pt1, e_pt2);
                           edges_added.push_back(edge);
                           points.push_back(e_pt1);
                           points.push_back(e_pt2);
                        }
                        else
                        {
                                for (size_t j = 0; j < edges_added.size(); j++)
                                {
                                        auto edge_compare = edges_added[j];

                                        RCLCPP_INFO(node_->get_logger(), "%zd/%zd", j, edges_added.size() - 1);
                                        RCLCPP_INFO(node_->get_logger(), "edge_new");
                                        RCLCPP_INFO(node_->get_logger(), "pt1 => index: %d, coord: (%.1f, %.1f)", edge.pt1_index, edge.pt1.first, edge.pt1.second);
                                        RCLCPP_INFO(node_->get_logger(), "pt2 => index: %d, coord: (%.1f, %.1f)", edge.pt2_index, edge.pt2.first, edge.pt2.second);
                                        RCLCPP_INFO(node_->get_logger(), "");
                                        RCLCPP_INFO(node_->get_logger(), "edge_old");
                                        RCLCPP_INFO(node_->get_logger(), "pt1 => index: %d, coord: (%.1f, %.1f)", edge_compare.pt1_index, edge_compare.pt1.first, edge_compare.pt1.second);
                                        RCLCPP_INFO(node_->get_logger(), "pt2 => index: %d, coord: (%.1f, %.1f)", edge_compare.pt2_index, edge_compare.pt2.first, edge_compare.pt2.second);
                                        RCLCPP_INFO(node_->get_logger(), "");

                                        if (is_neighbor(edge, edge_compare, dis_thr))
                                        {
                                                RCLCPP_INFO(node_->get_logger(), "is neighbor: satisfied");
                                                std::vector<Edge> edges_tmp;
                                                process_intersected_edges(edge, edge_compare, edges_tmp, points);
                                                merge_edges(edges_added, j, edges_tmp);
                                                break; // edge被添加后，要退出edges_add的for循环
                                        }
                                        else
                                        {
                                                RCLCPP_INFO(node_->get_logger(), "is neighbor: not satisfied");
                                                if (j == edges_added.size() - 1)
                                                {
                                                        RCLCPP_INFO(node_->get_logger(), "all old edges are not neighbored with the new edge, just push new edge's two points into points vector");
                                                        
                                                        EnhancedPoint e_pt1, e_pt2;
                           
                                                        e_pt1.index = edge.pt1_index;
                                                        e_pt1.visited = false;
                                                        e_pt1.coord = edge.pt1;

                                                        e_pt2.index = edge.pt2_index;
                                                        e_pt2.visited = false;
                                                        e_pt2.coord = edge.pt2;
                                                        
                                                        add_adjacent_relation(e_pt1, e_pt2);
                                                        edges_added.push_back(edge);
                                                        points.push_back(e_pt1);
                                                        points.push_back(e_pt2);
                                                        break; // 最后一个也不相交要break出来
                                                }
                                                continue;
                                        }
                                }
                        }

                        // debug
                        RCLCPP_INFO(node_->get_logger(), "");
                        RCLCPP_INFO(node_->get_logger(), "---------- print points info after loop %zd ----------", i);
                        for (size_t j = 0; j < points.size(); j++)
                        {
                                auto point = points[j];
                                RCLCPP_INFO(node_->get_logger(), "index %d", point.index);
                                // RCLCPP_INFO(node_->get_logger(), "visited %s", point.visited?"true":"false");
                                RCLCPP_INFO(node_->get_logger(), "point: (%.1f, %.1f)", point.coord.first, point.coord.second);
                                std::stringstream ss;
                                ss << "[ ";
                                for (size_t k = 0; k < point.adjacent_vec.size(); k++)
                                {
                                        ss << point.adjacent_vec[k];
                                        if (k == point.adjacent_vec.size() - 1)
                                        {
                                                ss << " ";
                                        }
                                        else
                                        {
                                                ss << ", ";
                                        }
                                }
                                ss << "]";
                                RCLCPP_INFO(node_->get_logger(), "adjacent: %s", ss.str().c_str());
                                RCLCPP_INFO(node_->get_logger(), "");
                        }
                }
        }

        void GenerateModel::merge_edges(std::vector<Edge>& edges_origin, const int& index_compare, const std::vector<Edge>& edges_tmp)
        {
                RCLCPP_INFO(node_->get_logger(), "start merge edges, edges_compare size: %zd, index_compare: %d, edges_tmp size: %zd", edges_origin.size(), index_compare, edges_tmp.size());
                assert_(index_compare < edges_origin.size(), "index_compare shall less than vector edges_origin's size.");
                assert_(index_compare >= 0, "index_compare shall >= 0.");
                edges_origin.erase(edges_origin.begin() + index_compare);
                assert_(edges_tmp.size() > 0, "edges_tmp's size shall > 0.");
                for (size_t i = 0; i < edges_tmp.size(); i++)
                {
                        edges_origin.push_back(edges_tmp[i]);
                }
                RCLCPP_INFO(node_->get_logger(), "after merge edges, print new edges");
                print_edges(edges_origin);
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

        double GenerateModel::compute_distance_of_two_edges(const Edge& edge1, const Edge& edge2)
        {
                auto pt11 = edge1.pt1;
                auto pt12 = edge1.pt2;
                auto pt21 = edge2.pt1;
                auto pt22 = edge2.pt2;
                double dis1 = point_to_line_distance_smart(pt11.first, pt11.second, pt21.first, pt21.second, pt22.first, pt22.second);
                double dis2 = point_to_line_distance_smart(pt12.first, pt12.second, pt21.first, pt21.second, pt22.first, pt22.second);
                double dis3 = point_to_line_distance_smart(pt21.first, pt21.second, pt11.first, pt11.second, pt12.first, pt12.second);
                double dis4 = point_to_line_distance_smart(pt22.first, pt22.second, pt11.first, pt11.second, pt12.first, pt12.second);
                return std::min({dis1, dis2, dis3, dis4});
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
                        // return (distance(px, py, x1, y1) + distance(px, py, x2, y2)) / 2.0;
                        return std::min(distance(px, py, x1, y1), distance(px, py, x2, y2));
                }
        }
        
        // 相交的两种情况
        // 1、顶点和顶点相交  => 相交的两个顶点合并为一个顶点，取两个顶点的中心位置为两条边的新顶点，为采用edge1的相交顶点索引值
        // 2、顶点和边体相交  => 在边体上取交点做为新顶点，新顶点采用相交顶点的索引，相交边体拆分成两条新边。
        // edge1是需要新增加的边,edge2是已经增加的边。
        void GenerateModel::process_intersected_edges(Edge& edge_new, Edge& edge_old, std::vector<Edge>& edges_tmp, std::vector<EnhancedPoint>& points)
        {
                RCLCPP_INFO(node_->get_logger(), "process_intersected_edges");
                
                edges_tmp.clear();
                Point pt_intersection;  // 相交的顶点
                Edge edge_intersection;         // 相交的边体
                Point pt_intersection_2; // 顶点和顶点相交时的另一个顶点

                // step 1: 找到相交的顶点
                std::vector<Point> points_origin_vec;
                size_t index_intersection; // 在points_origin_vec中交点的索引值
                points_origin_vec.push_back(edge_new.pt1);
                points_origin_vec.push_back(edge_new.pt2);
                points_origin_vec.push_back(edge_old.pt1);
                points_origin_vec.push_back(edge_old.pt2);
                double dis_min = std::numeric_limits<double>::max();
                for (size_t i = 0; i < points_origin_vec.size(); i++)
                {
                        size_t point_index = i;
                        size_t line_pt1_index, line_pt2_index;

                        line_pt1_index = i < 2 ? 2 : 0;
                        line_pt2_index = i < 2 ? 3 : 1;

                        Point point = points_origin_vec[point_index];
                        Point line_pt1 = points_origin_vec[line_pt1_index];
                        Point line_pt2 = points_origin_vec[line_pt2_index];
                        double dis_point_to_line = point_to_line_distance_smart(point.first, point.second, line_pt1.first, line_pt1.second, line_pt2.first, line_pt2.second);
                        if (dis_point_to_line < dis_min)
                        {
                                dis_min = dis_point_to_line;
                                index_intersection = point_index;
                                edge_intersection = point_index > 1 ? edge_new : edge_old;
                                pt_intersection = points_origin_vec[index_intersection];
                        }
                }
                
                RCLCPP_INFO(node_->get_logger(), "");
                RCLCPP_INFO(node_->get_logger(), "intersecting point");
                RCLCPP_INFO(node_->get_logger(), "dis_min: %.2f", dis_min);
                RCLCPP_INFO(node_->get_logger(), "in points_origin_vec, the intersection_index : %d, coord: (%.1f, %.1f)", index_intersection, pt_intersection.first, pt_intersection.second);
                RCLCPP_INFO(node_->get_logger(), "edge_intersection => pt1 coord: (%.1f, %.1f), pt2 coord: (%.1f, %.1f)", 
                        edge_intersection.pt1.first, edge_intersection.pt1.second,
                        edge_intersection.pt2.first, edge_intersection.pt2.second);
                RCLCPP_INFO(node_->get_logger(), "");

                // step2 判断相交类型是点与点相交还是点与边体相交。
                double dis_pt_pt1, dis_pt_pt2; // 选出的相交顶点到边体两个顶点的距离
                dis_pt_pt1 = distance(pt_intersection.first, pt_intersection.second, edge_intersection.pt1.first, edge_intersection.pt1.second);
                dis_pt_pt2 = distance(pt_intersection.first, pt_intersection.second, edge_intersection.pt2.first, edge_intersection.pt2.second);
                NeighborType intersection_type; // 0: 两个顶点相交， 1: 顶点与边体相交
                if (dis_pt_pt1 == dis_min)
                {
                        intersection_type = NeighborType::POINT_POINT;
                        pt_intersection_2 = edge_intersection.pt1;
                        RCLCPP_INFO(node_->get_logger(), "NeighborType: POINT_POINT");
                        RCLCPP_INFO(node_->get_logger(), "another intersection point: (%.1f, %.1f)", pt_intersection_2.first, pt_intersection_2.second);
                }
                else if (dis_pt_pt2 == dis_min)
                {
                        intersection_type = NeighborType::POINT_POINT;
                        pt_intersection_2 = edge_intersection.pt2;
                        RCLCPP_INFO(node_->get_logger(), "NeighborType: POINT_POINT");
                        RCLCPP_INFO(node_->get_logger(), "another intersection point: (%.1f, %.1f)", pt_intersection_2.first, pt_intersection_2.second);
                }
                else
                {
                        intersection_type = NeighborType::POINT_LINE;
                        RCLCPP_INFO(node_->get_logger(), "NeighborType: POINT_LINE");
                }

                // step3 合并顶点或拆分edge
                switch(intersection_type)
                {
                        case NeighborType::POINT_POINT:  // 两个顶点相交=>合并顶点
                        {
                                RCLCPP_INFO(node_->get_logger(), "process case NeighborType::POINT_POINT");
                                Point intersection_point_new;
                                intersection_point_new.first = (pt_intersection.first + pt_intersection_2.first) / 2.0;
                                intersection_point_new.second = (pt_intersection.second + pt_intersection_2.second) / 2.0;

                                EnhancedPoint e_edge1_pt1, e_edge1_pt2, e_edge2_pt1, e_edge2_pt2;
                                
                                e_edge1_pt1.index = edge_new.pt1_index;
                                e_edge1_pt1.coord = edge_new.pt1;
                                e_edge1_pt1.visited = false;
                                
                                //  更新两个edge的交点的坐标值、顶点间的相邻关系
                                int intersection_index_new; // 用于保存合并的顶点的索引值
                                EnhancedPoint e_edge_new_another_point;

                                if (index_intersection > 1) // 初始的相交顶点取在edge_old上,对edge_old的处理只需更新points中对应的点的坐标值。
                                {
                                        // 更新edge_old的交点的坐标值 => 替换相交的顶点
                                        if (edge_old.pt1 == pt_intersection)
                                        {
                                                edge_old.pt1 = intersection_point_new;
                                                intersection_index_new = edge_old.pt1_index;

                                                // 替换points中的坐标
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_old.pt1_index)
                                                        {
                                                                points[i].coord = intersection_point_new;
                                                        }
                                                }
                                        }
                                        else
                                        {
                                                edge_old.pt2 = intersection_point_new;
                                                intersection_index_new = edge_old.pt2_index;
                                                // 替换points中的坐标
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_old.pt2_index)
                                                        {
                                                                points[i].coord = intersection_point_new;
                                                        }
                                                }
                                        }

                                        // 更新edge_new的交点的坐标值 => 替换相交的顶点
                                        if (edge_new.pt1 == pt_intersection_2)  // 交点是pt1
                                        {
                                                edge_new.pt1 = intersection_point_new;
                                                edge_new.pt1_index = intersection_index_new;
                                                // 增加另一个点到points中
                                                e_edge_new_another_point.coord = edge_new.pt2;
                                                e_edge_new_another_point.index = edge_new.pt2_index;
                                                e_edge_new_another_point.visited = false;
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_new.pt1_index)
                                                        {
                                                                add_adjacent_relation(points[i], e_edge_new_another_point);
                                                                points.push_back(e_edge_new_another_point);
                                                        }
                                                }
                                                
                                        }
                                        else  // 交点是pt2
                                        {
                                                edge_new.pt2 = intersection_point_new;
                                                edge_new.pt2_index = intersection_index_new;
                                                // 增加另一个点到points中
                                                e_edge_new_another_point.coord = edge_new.pt1;
                                                e_edge_new_another_point.index = edge_new.pt1_index;
                                                e_edge_new_another_point.visited = false;
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_new.pt2_index)
                                                        {
                                                                add_adjacent_relation(points[i], e_edge_new_another_point);
                                                                points.push_back(e_edge_new_another_point);
                                                        }
                                                }
                                        }
                                }
                                else // 初始的相交顶点取在edge_new上
                                {
                                        // 更新edge_old的交点的坐标值  => 替换相交的顶点
                                        if (edge_old.pt1 == pt_intersection_2) // 交点是pt1
                                        {
                                                edge_old.pt1 = intersection_point_new;                                                
                                                intersection_index_new = edge_old.pt1_index;

                                                // 替换points中的坐标
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_old.pt1_index)
                                                        {
                                                                points[i].coord = intersection_point_new;
                                                        }
                                                }
                                        }
                                        else // 交点是pt2
                                        {
                                                edge_old.pt2 = intersection_point_new;
                                                intersection_index_new = edge_old.pt2_index;
                                                // 替换points中的坐标
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_old.pt2_index)
                                                        {
                                                                points[i].coord = intersection_point_new;
                                                        }
                                                }
                                        }

                                        // 更新edge_new的交点的坐标值  => 替换相交的顶点
                                        if (edge_new.pt1 == pt_intersection)
                                        {
                                                edge_new.pt1 = intersection_point_new;                                                
                                                edge_new.pt1_index = intersection_index_new;
                                                
                                                // 增加另一个点到points中
                                                e_edge_new_another_point.coord = edge_new.pt2;
                                                e_edge_new_another_point.index = edge_new.pt2_index;
                                                e_edge_new_another_point.visited = false;
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_new.pt1_index)
                                                        {
                                                                add_adjacent_relation(points[i], e_edge_new_another_point);
                                                                points.push_back(e_edge_new_another_point);
                                                        }
                                                }
                                        }
                                        else
                                        {
                                                edge_new.pt2 = intersection_point_new;                                                 
                                                edge_new.pt2_index = intersection_index_new;
                                                // 增加另一个点到points中
                                                e_edge_new_another_point.coord = edge_new.pt1;
                                                e_edge_new_another_point.index = edge_new.pt1_index;
                                                e_edge_new_another_point.visited = false;
                                                for (size_t i = 0; i < points.size(); i++)
                                                {
                                                        if (points[i].index == edge_new.pt2_index)
                                                        {
                                                                add_adjacent_relation(points[i], e_edge_new_another_point);
                                                                points.push_back(e_edge_new_another_point);
                                                        }
                                                }
                                        }
                                }

                                // 保存两个新edge到edges_tmp中
                                edges_tmp.push_back(edge_new);
                                edges_tmp.push_back(edge_old);
                                break;
                        }
                        case NeighborType::POINT_LINE:   // 顶点和边体相交
                        {
                                RCLCPP_INFO(node_->get_logger(), "process case NeighborType::POINT_LINE");
                                // 1、找到在边体上的交点
                                auto point_new = find_neareast_point(pt_intersection.first, pt_intersection.second, 
                                edge_intersection.pt1.first, edge_intersection.pt1.second,
                                edge_intersection.pt2.first, edge_intersection.pt2.second);
                                RCLCPP_INFO(node_->get_logger(), "intersection point: (%.1f, %.1f)", point_new.first, point_new.second);

                                // 2、更新points
                                EnhancedPoint e_pt_new_intersection; // 交点
                                e_pt_new_intersection.coord = point_new;
                                e_pt_new_intersection.visited = false;
                                // e_pt_new_intersection.index = index++;

                                // 3、判断edge_intersection是已有edge还是新edge
                                bool body_is_old_edge = edge_intersection.pt1_index == edge_old.pt1_index;

                                if (body_is_old_edge)  // 除了交点，只需要增加一个点（edge_new的另一个点）
                                {
                                        RCLCPP_INFO(node_->get_logger(), "edge_old is the intersection edge");
                                        e_pt_new_intersection.index = pt_intersection ==  edge_new.pt1 ? edge_new.pt1_index : edge_new.pt2_index;
                                        // 赋值新edge_new的另一个点
                                        auto point_tmp = pt_intersection ==  edge_new.pt1 ? edge_new.pt2 : edge_new.pt1;
                                        EnhancedPoint e_new_pt2;
                                        e_new_pt2.coord = point_tmp;
                                        e_new_pt2.visited = false;
                                        e_new_pt2.index = pt_intersection ==  edge_new.pt1 ? edge_new.pt2_index : edge_new.pt1_index;
                                        // 删除原edge两个点的相邻关系,新增与交点的相邻关系
                                        int index_edge_old_pt1 = -1, index_edge_old_pt2 = -1;
                                        for (size_t i = 0; i < points.size(); i++)                                      
                                        {
                                                if (points[i].index == edge_old.pt1_index)
                                                {
                                                        index_edge_old_pt1 = i;
                                                }
                                                else if (points[i].index == edge_old.pt2_index)
                                                {
                                                        index_edge_old_pt2 = i;
                                                }
                                        }

                                        assert_(index_edge_old_pt1 != -1, "index_edge_old_pt1 has invalid value -1");
                                        assert_(index_edge_old_pt2 != -1, "index_edge_old_pt2 has invalid value -1");

                                        Edge edge_new_1, edge_new_2, edge_new_3;

                                        edge_new_1.pt1_index = points[index_edge_old_pt1].index;
                                        edge_new_1.pt1 = points[index_edge_old_pt1].coord;
                                        edge_new_1.pt2_index = e_pt_new_intersection.index;
                                        edge_new_1.pt2 = e_pt_new_intersection.coord;

                                        edge_new_2.pt1_index = points[index_edge_old_pt2].index;
                                        edge_new_2.pt1 = points[index_edge_old_pt2].coord;
                                        edge_new_2.pt2_index = e_pt_new_intersection.index;
                                        edge_new_2.pt2 = e_pt_new_intersection.coord;

                                        edge_new_3.pt1_index = e_pt_new_intersection.index;
                                        edge_new_3.pt1 = e_pt_new_intersection.coord;
                                        edge_new_3.pt2_index = e_new_pt2.index;
                                        edge_new_3.pt2 = e_new_pt2.coord;

                                        edges_tmp.push_back(edge_new_1);
                                        edges_tmp.push_back(edge_new_2);
                                        edges_tmp.push_back(edge_new_3);

                                        remove_adjacent_relation(points[index_edge_old_pt1], points[index_edge_old_pt2]);
                                        add_adjacent_relation(e_pt_new_intersection, points[index_edge_old_pt1]);
                                        add_adjacent_relation(e_pt_new_intersection, points[index_edge_old_pt2]);                                        
                                        add_adjacent_relation(e_pt_new_intersection, e_new_pt2); // 增加新edge的另一个点和交点的相邻关系

                                        points.push_back(e_pt_new_intersection);
                                        points.push_back(e_new_pt2);
                                }
                                else // 除了交点，需要增加两个点（新edge的两个点）, edge_old的交点替换成新交点
                                {
                                        RCLCPP_INFO(node_->get_logger(), "edge_new is the intersection edge");
                                        // 找到edge_old的交点在points中的索引值
                                        int e_edge_old_intersection_index = -1;
                                        std::vector<int> e_pt_sub_adj; // 保留edge_old中交点的相邻关系vector
                                        for (size_t i = 0; i < points.size(); i++)
                                        {
                                                if (points[i].coord == pt_intersection)
                                                {
                                                        e_edge_old_intersection_index = points[i].index;
                                                        e_pt_sub_adj = points[i].adjacent_vec;
                                                        break;
                                                }
                                        }
                                        assert_(e_edge_old_intersection_index != -1, "e_edge_old_intersection_index has invalid value -1.");

                                        // 恢复交点的index和相邻关系vector
                                        e_pt_new_intersection.index = e_edge_old_intersection_index;
                                        e_pt_new_intersection.adjacent_vec = e_pt_sub_adj;

                                        // 赋值新增加的两个点
                                        EnhancedPoint e_new_edge_pt1, e_new_edge_pt2;
                                        
                                        e_new_edge_pt1.coord = edge_new.pt1;
                                        e_new_edge_pt1.visited = false;
                                        e_new_edge_pt1.index = edge_new.pt1_index;

                                        e_new_edge_pt2.coord = edge_new.pt2;
                                        e_new_edge_pt2.visited = false;
                                        e_new_edge_pt2.index = edge_new.pt2_index;

                                        Edge edge_new_1, edge_new_2, edge_new_3; 
                                        edge_new_1.pt1_index = e_new_edge_pt1.index;
                                        edge_new_1.pt1 = e_new_edge_pt1.coord;
                                        edge_new_1.pt2_index = e_pt_new_intersection.index;
                                        edge_new_1.pt2 = e_pt_new_intersection.coord;
                                        
                                        edge_new_2.pt1_index = e_new_edge_pt2.index;
                                        edge_new_2.pt1 = e_new_edge_pt2.coord;
                                        edge_new_2.pt2_index = e_pt_new_intersection.index;
                                        edge_new_2.pt2 = e_pt_new_intersection.coord;

                                        // 找到edge_old中非相交的点。
                                        edge_new_3.pt1_index = e_pt_new_intersection.index;
                                        edge_new_3.pt1 = e_pt_new_intersection.coord;
                                        if (pt_intersection == edge_old.pt1) // 交点是pt1, 赋值pt2
                                        {
                                                edge_new_3.pt2_index = edge_old.pt2_index;
                                                edge_new_3.pt2 = edge_old.pt2;
                                        }
                                        else // 交点是pt2, 赋值pt1
                                        {
                                                edge_new_3.pt1_index = edge_old.pt1_index;
                                                edge_new_3.pt1 = edge_old.pt1;
                                        }

                                        edges_tmp.push_back(edge_new_1);
                                        edges_tmp.push_back(edge_new_2);
                                        edges_tmp.push_back(edge_new_3);

                                        // 添加 edge_new两个点和交点的相邻关系
                                        add_adjacent_relation(e_new_edge_pt1, e_pt_new_intersection);
                                        add_adjacent_relation(e_new_edge_pt2, e_pt_new_intersection);

                                        // 替换原edge中的相交顶点
                                        for (size_t i = 0; i < points.size(); i++)
                                        {
                                             if (points[i].coord == pt_intersection)
                                             {
                                                points[i] = e_pt_new_intersection;
                                             }   
                                        }

                                        points.push_back(e_new_edge_pt1);
                                        points.push_back(e_new_edge_pt2);
                                }

                                break;
                        }
                }
        }

        // 找到某一个点在线段上离它最近的点  
        Point GenerateModel::find_neareast_point(double px, double py, double seg1x, double seg1y, double seg2x, double seg2y)
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

        void GenerateModel::sort_edges(std::vector<Edge>& edges)
        {
                // 先初始化index、weight、adjacent
                int index = 0;
                for (auto & edge: edges)
                {
                        edge.index = index++;
                }

                // 获取edge的相邻关系
                std::unordered_map<int, std::vector<int>> adj;
                for (size_t i = 0; i < edges.size() - 1; i++)
                {
                        for (size_t j = i + 1;j < edges.size(); j++)
                        {
                                auto &edge1 = edges[i];
                                auto &edge2 = edges[j];
                                if (is_neighbor(edge1, edge2, this->dis_thr_))
                                {
                                        adj[i].push_back(j);
                                        adj[j].push_back(i);
                                }
                        }
                }

                // 根据相邻关系，按BFS排序。
                std::vector<int>order;
                std::queue<int> q;
                int root = 0; // 根节点

                std::unordered_map<int, bool> visited;
                std::unordered_map<int, int> parent;
                for (size_t i = 0; i < edges.size(); i++)
                {
                        visited[i] = false;
                }

                //初始化根节点
                q.push(root);
                visited[root] = true;
                parent[root] = -1;
                order.push_back(root);

                // BFS 遍历树结构
                while(!q.empty())
                {
                        int u = q.front();
                        q.pop();

                        // 遍历相邻节点
                        for (int v : adj.at(u))
                        {
                                if (!visited[v])
                                {
                                        visited[v] = true;
                                        parent[v] = u; // 记录夫节点
                                        order.push_back(v);
                                        q.push(v);
                                }
                        }
                }

                RCLCPP_INFO(node_->get_logger(), "order =>  %s", vector_to_string(order));

                const auto edges_clone = edges;
                edges.clear();
                for (size_t i = 0; i < order.size(); i++)
                {
                        auto order_number = order[i];
                        for (size_t j = 0; j < edges_clone.size(); j++)
                        {
                                if (edges_clone[j].index == order_number)
                                {
                                        edges.push_back(edges_clone[j]);
                                        break;
                                }
                        }
                }
        }
        

} // end of namespace


