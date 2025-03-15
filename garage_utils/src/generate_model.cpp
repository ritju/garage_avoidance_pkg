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
                int index = 0;
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
                                        process_intersected_edges(edge, edge_adjacent, edges);
                                }
                        }
                        else
                        {
                                RCLCPP_WARN(node_->get_logger(), "edge %zd has 0 adjacent edge.[(%f, %f), (%f, %f)]",
                                        i, edge.pt1.first, edge.pt1.second, edge.pt2.first, edge.pt2.second );
                        }
                }

        }

        bool GenerateModel::is_neighbor(const Edge& e1, const Edge& e2, double dis_thr)
        {
                double edges_distance = compute_distance_of_two_edges(e1.pt1, e2.pt2, e1.pt1, e2.pt2);
                return edges_distance < dis_thr;
        }

        bool GenerateModel::has_neighbor(const std::vector<Edge> edges, const Edge& edge, std::vector<std::pair<NeighborType,Edge>>& edges_neighbor_vec, double dis_thr)
        {
              bool ret = false;
              edges_neighbor_vec.clear();
              for (size_t i = 0; i < edges.size(); i++)
              {
                auto edge_compare = edges[i];
                if (is_neighbor(edge, edge_compare, dis_thr))  
                {
                        ret = true;
                        // 填充存储相邻edges对及相邻关系类型的vector
                }
                else
                {
                        continue;
                }
              }

              return ret;
        }

        void GenerateModel::generate_points(std::vector<EnhancedPoint>& points, const std::vector<Edge>& edges)
        {
                // 先清空points
                points.clear();

                std::vector<Edge> edges_added; // 用来存放已经添加过的edges
                int index = 0;

                for (size_t i = 0; i < edges.size(); i++)
                {
                        auto edge = edges[i];
                        if (edges_added.size() == 0) // 第一次遍历时
                        {
                           edges_added.push_back(edge);
                           EnhancedPoint pt1, pt2;
                           
                           pt1.edge_index = i;
                           pt1.index = index++;
                           pt1.visited = false;
                           pt1.coord = edge.pt1;

                           pt2.edge_index = i;
                           pt2.index = index++;
                           pt2.visited = false;
                           pt2.coord = edge.pt2;

                           add_adjacent_relation(pt1, pt2);
                           pt1.adjacent_vec.push_back(pt2.index);
                           pt2.adjacent_vec.push_back(pt1.index);

                           edges_added.push_back(edge);
                        }
                        else
                        {

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
        
        // 相交的两种情况
        // 1、顶点和顶点相交  => 相交的两个顶点合并为一个顶点，取两个顶点的中心位置为两条边的新顶点，为采用edge1的相交顶点索引值
        // 2、顶点和边体相交  => 在边体上取交点做为新顶点，新顶点采用相交顶点的索引，相交边体拆分成两条新边。
        void GenerateModel::process_intersected_edges(Edge& edge1, Edge& edge2, std::vector<Edge>& edges)
        {
                Point pt_intersection;  // 相交的顶点
                Edge edge_body;         // 相交的边体
                Point pt_intersection_2; // 顶点和顶点相交时的另一个顶点

                // step 1: 找到相交的顶点
                size_t index_intersection;
                std::vector<Point> points_origin_vec;
                points_origin_vec.push_back(edge1.pt1);
                points_origin_vec.push_back(edge1.pt2);
                points_origin_vec.push_back(edge2.pt1);
                points_origin_vec.push_back(edge2.pt2);
                double dis_min = std::numeric_limits<double>::max();
                for (size_t i = 0; i < points_origin_vec.size(); i++)
                {
                        size_t point_index = i;
                        size_t line_pt1_index, line_pt2_index, point_intersection_index;

                        line_pt1_index = i < 2 ? 2 : 0;
                        line_pt2_index = i < 2 ? 3 : 1;

                        Point point = points_origin_vec[i];
                        Point line_pt1 = points_origin_vec[line_pt1_index];
                        Point line_pt2 = points_origin_vec[line_pt2_index];
                        double dis_point_to_line = point_to_line_distance_smart(point.first, point.second, line_pt1.first, line_pt1.second, line_pt2.first, line_pt2.second);
                        if (dis_point_to_line < dis_min)
                        {
                                dis_min = dis_point_to_line;
                                index_intersection = i;
                                edge_body = i > 1 ? edge1 : edge2;
                                pt_intersection = points_origin_vec[index_intersection];
                        }
                }

                // step2 判断相交类型是点与点相交还是点与边体相交。
                double dis_pt_pt1, dis_pt_pt2; // 选出的相交顶点到边体两个顶点的距离
                dis_pt_pt1 = distance(pt_intersection.first, pt_intersection.second, edge_body.pt1.first, edge_body.pt1.second);
                dis_pt_pt2 = distance(pt_intersection.first, pt_intersection.second, edge_body.pt2.first, edge_body.pt2.second);
                int intersection_type; // 0: 两个顶点相交， 1: 顶点与边体相交
                if (dis_pt_pt1 == dis_min)
                {
                        intersection_type = 0;
                        pt_intersection_2 = edge_body.pt1;
                }
                else if (dis_pt_pt2 == dis_min)
                {
                        intersection_type = 0;
                        pt_intersection_2 = edge_body.pt2;
                }
                else
                {
                        intersection_type = 1;
                }

                // step3 合并顶点或拆分edge
                if (!intersection_type) // 两个顶点相交=>合并顶点
                {
                        Point new_point;
                        new_point.first = (pt_intersection.first + pt_intersection.first) / 2.0;
                        new_point.second = (pt_intersection.second + pt_intersection.second) / 2.0;
                        
                        // 替换 new_point
                        // todo 更新edge的其它信息
                        if (index_intersection > 1) // 初始的相交顶点取在edge2上
                        {
                                // 更新edge2
                                if (edge2.pt1.first == pt_intersection.first && edge2.pt1.second == pt_intersection.second)
                                {
                                        edge2.pt1 = new_point;
                                }
                                else
                                {
                                        edge2.pt2 = new_point;
                                }
                                // 更新edge1
                                if (edge1.pt1.first == pt_intersection_2.first && edge1.pt1.second == pt_intersection_2.second)
                                {
                                        edge1.pt1 = new_point;
                                }
                                else
                                {
                                        edge1.pt2 = new_point;
                                }
                        }
                        else // 初始的相交顶点取在edge1上
                        {
                                // 更新edge2
                                if (edge2.pt1.first == pt_intersection_2.first && edge2.pt1.second == pt_intersection_2.second)
                                {
                                        edge2.pt1 = new_point;
                                }
                                else
                                {
                                        edge2.pt2 = new_point;
                                }
                                // 更新edge1
                                if (edge1.pt1.first == pt_intersection.first && edge1.pt1.second == pt_intersection.second)
                                {
                                        edge1.pt1 = new_point;
                                }
                                else
                                {
                                        edge1.pt2 = new_point;
                                }
                        }
                                            
                }
                else // 顶点和边体相交
                {
                        // 1、找到在边体上的交点
                        auto point_new = find_neareast_point(pt_intersection.first, pt_intersection.second, 
                                edge_body.pt1.first, edge_body.pt1.second,
                                edge_body.pt2.first, edge_body.pt2.second);

                        // 2、更新边体edge为两个新的edge
                        Edge edge_new_1, edge_new_2;
                        edge_new_1.pt1 = edge_body.pt1;
                        edge_new_1.pt2 = point_new;
                        edge_new_2.pt1 = point_new;
                        edge_new_2.pt2 = edge_body.pt2;
                        // 添加新edges到vector中
                        edges.push_back(edge_new_1);
                        edges.push_back(edge_new_2);                        
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
        

} // end of namespace


