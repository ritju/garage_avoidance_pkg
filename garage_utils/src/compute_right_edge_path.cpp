#include "garage_utils/compute_right_edge_path.hpp"

namespace garage_utils_pkg
{
        ComputeRightEdgePathActionServer::ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options)
                : Node("compute_right_edge_path_action_server", options)
        {
                RCLCPP_INFO(get_logger(), "compute_right_edge_path_action_server constructor.");
                using namespace std::placeholders;

                init_params();

                // init tf2
                this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);                
                
                std::vector<geometry_msgs::msg::Polygon>().swap(polygons_);

                auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                auto pub_ops = rclcpp::PublisherOptions();
                pub_ops.callback_group = callback_group;
                path_pub_ = this->create_publisher<nav_msgs::msg::Path>(this->path_topic_name_, rclcpp::QoS{1}.reliable(), pub_ops);

                this->action_server_ = rclcpp_action::create_server<ComputeRightEdgePath>(
                this,
                "compute_right_edge_path_action_server",
                std::bind(&ComputeRightEdgePathActionServer::handle_goal, this, _1, _2),
                std::bind(&ComputeRightEdgePathActionServer::handle_cancel, this, _1),
                std::bind(&ComputeRightEdgePathActionServer::handle_accepted, this, _1));
        }
        
        ComputeRightEdgePathActionServer::~ComputeRightEdgePathActionServer()
        {
                delete path_searcher_;
                delete model_generator_;
        }

        void ComputeRightEdgePathActionServer::init_params()
        {
                this->declare_parameter<std::string>("path_topic_name", "garage_path");
                this->declare_parameter<double>("dis_thr", 2.5);
                this->declare_parameter<double>("resolution", 2.0);

                this->dis_thr_ = this->get_parameter_or("dis_thr", 2.5);
                this->path_topic_name_ = this->get_parameter_or("path_topic_name", std::string("garage_path"));

                RCLCPP_DEBUG(get_logger(), "------------ list params ------------ ");
                RCLCPP_DEBUG(get_logger(), "dist_thr: %f", this->dis_thr_);
                RCLCPP_DEBUG(get_logger(), "path_topic_name: %s", this->path_topic_name_.c_str());
        }

        void ComputeRightEdgePathActionServer::print_polygons(const std::vector<geometry_msgs::msg::Polygon> polygons)
        {
                for (size_t i = 0; i < polygons.size(); i++)
                {
                        std::stringstream ss;
                        ss << "[";
                        geometry_msgs::msg::Polygon polygon = polygons[i];
                        for (size_t j = 0; j < polygon.points.size(); j++)
                        {
                                geometry_msgs::msg::Point32 point = polygon.points[j];
                                ss << "(" <<  point.x << ", " << point.y << ")";
                                if (j != (polygon.points.size() - 1))
                                {
                                        ss << ", ";
                                }
                        }
                        ss << "]";
                        RCLCPP_INFO(get_logger(), "polygon %zd: %s", i, ss.str().c_str());
                }
        }

        void ComputeRightEdgePathActionServer::print_rect(const EnhancedRect& rect)
        {
                auto vertices = rect.vertices;
                for (size_t i = 0; i < vertices.size(); i++)
                {
                        RCLCPP_INFO(get_logger(), "vertices %zd: (%f, %f, %f)", i, vertices[i].x, vertices[i].y, vertices[i].angle);
                }
                RCLCPP_INFO(get_logger(), "middle_long_line: (%f, %f), (%f, %f)", 
                        rect.middle_long_line[0].first, rect.middle_long_line[0].second,
                        rect.middle_long_line[1].first, rect.middle_long_line[1].second);
        }

        rclcpp_action::GoalResponse ComputeRightEdgePathActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const ComputeRightEdgePath::Goal> goal)
        {
                (void)uuid;

                if (!this->model_generator_)
                {
                        RCLCPP_INFO(get_logger(), "init [ model_generator_, path_searcher_, path_generator_ ] .");
                        auto node_ptr = shared_from_this();
                        this->model_generator_ = new GenerateModel(node_ptr);
                        this->path_searcher_ = new ShortestPathSearch(node_ptr);
                        this->path_generator_ = new GeneratePath(node_ptr);
                }

                RCLCPP_INFO(this->get_logger(), "Received goal request with %ld polygons, the pose of car (%f, %f).", 
                        goal->polygons.size(), goal->car_pose.pose.position.x, goal->car_pose.pose.position.y);

                // 保存car_pose和polygons
                this->car_pose_ = goal->car_pose;
                this->polygons_ = goal->polygons;

                // 打印 goal里的polygons
                RCLCPP_INFO(get_logger(), "print polygons in origin goal.");
                print_polygons(polygons_);

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;                
        }

        rclcpp_action::CancelResponse ComputeRightEdgePathActionServer::handle_cancel(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                
                (void)goal_handle;                
                return rclcpp_action::CancelResponse::ACCEPT;
        }

        void ComputeRightEdgePathActionServer::handle_accepted(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                using namespace std::placeholders;
                // this needs to return quickly to avoid blocking the executor, so spin up a new thread
                std::thread{std::bind(&ComputeRightEdgePathActionServer::execute, this, _1), goal_handle}.detach();
        }

        void ComputeRightEdgePathActionServer::execute(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                // define result
                auto result = std::make_shared<garage_utils_msgs::action::ComputeRightEdgePath::Result>();

                try
                {
                        // 检查goal的合法性
                        std::string reason;
                        if (!goal_checker(goal_handle->get_goal(), reason))
                        {
                                RCLCPP_INFO(this->get_logger(), "reject the goal, for reason: %s", reason.c_str());
                                auto result = std::make_shared<ComputeRightEdgePath::Result>();
                                result->reason = reason;
                                result->success = false;
                                goal_handle->abort(result);
                                return;
                        }
                        
                        RCLCPP_INFO(get_logger(), "Begin to compute path following right edge.");
                        const auto goal = goal_handle->get_goal();
                        auto feedback = std::make_shared<ComputeRightEdgePath::Feedback>();

                        // 根据car和机器人的位置，截断其所在的polygon
                        while(!get_map_robot_tf())
                        {
                                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for get tf of robot in map.");
                                std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
                        }

                        double robot_x, robot_y, car_x, car_y;
                        robot_x = map_robot_tf.getOrigin().getX();
                        robot_y = map_robot_tf.getOrigin().getY();
                        car_x = goal->car_pose.pose.position.x;
                        car_y = goal->car_pose.pose.position.y;

                        // tmp, !!!!!!!!!!!!!!! delete atfer test !!!!!!!!!!!!!!
                        // robot_x = 0.0;
                        // robot_y = 3.0;
                        
                        RCLCPP_INFO(get_logger(), "robot_pose: (%f, %f)", robot_x, robot_y);

                        size_t current_index = get_current_polygon_index(map_robot_tf, polygons_);
                        RCLCPP_INFO(get_logger(), "current_index: %ld", current_index);
                        auto polygon_first = this->polygons_[current_index];                
                        this->polygons_.erase(polygons_.begin() + current_index);

                        // 找到起点边框的middle_long_line
                        EnhancedRect enhanced_rect;
                        for (size_t j = 0; j < polygon_first.points.size(); j++)
                        {
                                Vertex vertex;
                                vertex.x = polygon_first.points[j].x;
                                vertex.y = polygon_first.points[j].y;
                                enhanced_rect.vertices.push_back(vertex);
                        }
                        model_generator_->sortVertices(enhanced_rect);
                        model_generator_->generate_middle_long_line(enhanced_rect);

                        RCLCPP_INFO(get_logger(), "print first enhanced rect info before cutting.");
                        print_rect(enhanced_rect);

                        // 对起点边框进行切割
                        // 找到两条长边框(和middle_long_line的夹角最小的两条边)
                        std::vector<std::pair<Point, Point>> edges;
                        for (size_t i = 0; i < enhanced_rect.vertices.size(); i++)
                        {
                                std::pair<Point, Point> edge;
                                edge.first.first = enhanced_rect.vertices[i].x;
                                edge.first.second = enhanced_rect.vertices[i].y;
                                edge.second.first = enhanced_rect.vertices[ (i + 1) % 4].x;
                                edge.second.second = enhanced_rect.vertices[(i + 1) % 4].y;
                                edges.push_back(edge);
                        }
                        // 把edges按与middle_long_line的夹角按从小到大的顺序排序
                        std::sort(edges.begin(), edges.end(), 
                                [enhanced_rect, this](std::pair<Point, Point>& edge1, std::pair<Point, Point>& edge2)
                                {
                                        return theta_between_two_edges(edge1.first, edge1.second,
                                                enhanced_rect.middle_long_line[0], enhanced_rect.middle_long_line[1]) 
                                                <
                                                theta_between_two_edges(edge2.first, edge2.second,
                                                enhanced_rect.middle_long_line[0], enhanced_rect.middle_long_line[1]) ;
                                });
                        auto long_line1 = edges[0];
                        auto long_line2 = edges[1];
                        RCLCPP_INFO(get_logger(), "long line1 => [ (%f, %f), (%f, %f) ]", 
                                long_line1.first.first, long_line1.first.second,
                                long_line1.second.first, long_line1.second.second);
                        RCLCPP_INFO(get_logger(), "long line2 => [ (%f, %f), (%f, %f) ]", 
                                long_line2.first.first, long_line2.first.second,
                                long_line2.second.first, long_line2.second.second);
                        
                        // 选取原有的两个顶点
                        Point p1_selected, p2_selected;

                        double l1_x = long_line1.second.first - long_line1.first.first;
                        double l1_y = long_line1.second.second - long_line1.first.second;
                        double l2_x = long_line2.second.first - long_line2.first.first;
                        double l2_y = long_line2.second.second - long_line2.first.second;
                        double car_robot_x = robot_x - car_x;
                        double car_robot_y = robot_y - car_y;

                        double dot1, dot2;

                        dot1 = l1_x * car_robot_x + l1_y * car_robot_y;
                        if (dot1 > 0)
                        {
                                p1_selected = long_line1.second;
                        }
                        else
                        {
                                p1_selected = long_line1.first;
                        }

                        dot2 = l2_x * car_robot_x + l2_y * car_robot_y;
                        if (dot2 > 0)
                        {
                                p2_selected = long_line2.second;
                        }
                        else
                        {
                                p2_selected = long_line2.first;
                        }

                        RCLCPP_INFO(get_logger(), "dot1: %f, dot2: %f", dot1, dot2);
                        RCLCPP_INFO(get_logger(), "p1_selected: (%f, %f)", p1_selected.first, p1_selected.second);
                        RCLCPP_INFO(get_logger(), "p2_selected: (%f, %f)", p2_selected.first, p2_selected.second);

                        // 找到切割的矩形的另外两个点
                        Point p3_selected, p4_selected;
                        p3_selected = find_neareast_point(robot_x, robot_y, long_line1.first.first, long_line1.first.second, long_line1.second.first, long_line1.second.second);
                        p4_selected = find_neareast_point(robot_x, robot_y, long_line2.first.first, long_line2.first.second, long_line2.second.first, long_line2.second.second);

                        RCLCPP_INFO(get_logger(), "p3_selected: (%f, %f)", p3_selected.first, p3_selected.second);
                        RCLCPP_INFO(get_logger(), "p4_selected: (%f, %f)", p4_selected.first, p4_selected.second);

                        // bug fix p1==p3 || p2==p4
                        if (p1_selected == p3_selected || p2_selected == p4_selected)
                        {
                                result->success = false;
                                result->path = nav_msgs::msg::Path();
                                result->poses = this->path_.poses;
                                goal_handle->abort(result);
                                RCLCPP_INFO(get_logger(), "Goal Failed, fix p1==p3 || p2==p4");
                                return;
                        }
                        
                        // 更新 polygon_first
                        polygon_first.points.clear();
                        geometry_msgs::msg::Point32 pt32;
                        
                        pt32.x = p1_selected.first;
                        pt32.y = p1_selected.second;
                        polygon_first.points.push_back(pt32);
                        
                        pt32.x = p2_selected.first;
                        pt32.y = p2_selected.second;
                        polygon_first.points.push_back(pt32);
                        
                        pt32.x = p3_selected.first;
                        pt32.y = p3_selected.second;
                        polygon_first.points.push_back(pt32);
                        
                        pt32.x = p4_selected.first;
                        pt32.y = p4_selected.second;
                        polygon_first.points.push_back(pt32);

                        this->polygons_.emplace(this->polygons_.begin(), polygon_first);

                        RCLCPP_INFO(get_logger(), "print polygons after cutting.");
                        print_polygons(this->polygons_);

                        // 1、generate model
                        std::vector<std::vector<Point>> rects;
                        for (size_t i = 0; i < this->polygons_.size(); i++)
                        {
                                auto polygon = polygons_[i];
                                std::vector<Point> rect;
                                for (size_t j = 0; j < polygon.points.size(); j++)
                                {
                                        Point point;
                                        point.first = polygon.points[j].x;
                                        point.second = polygon.points[j].y;
                                        rect.push_back(point);                                
                                }
                                rects.push_back(rect);
                        }

                        model_generator_->print_rects(rects);
                        model_generator_->process_(rects, dis_thr_);
                        auto points = model_generator_->get_points();

                        // 打印 model中的points
                        RCLCPP_INFO(get_logger(), "print result");
                        RCLCPP_INFO(get_logger(), "points size: %zd", points.size());
                        for (size_t i = 0; i < points.size(); i++)
                        {
                                auto point = points[i];
                                RCLCPP_INFO(get_logger(), "---------- points %zd ----------", i);
                                RCLCPP_INFO(get_logger(), "index: %d", point.index);
                                RCLCPP_INFO(get_logger(), "visited: %s", point.visited?"true":"false");
                                RCLCPP_INFO(get_logger(), "coord: (%f, %f)", point.coord.first, point.coord.second);
                                std::stringstream ss;
                                // RCLCPP_INFO(node->get_logger(), "adjacent_vec size: %zd", point.adjacent_vec.size());
                                for (size_t j = 0; j < point.adjacent_vec.size(); j++)
                                {
                                        ss << point.adjacent_vec[j] ;
                                        if (j == point.adjacent_vec.size() - 1)
                                        {
                                                ss << " ";
                                        }
                                        else
                                        {
                                                ss << ", ";
                                        }
                                }
                                RCLCPP_INFO(get_logger(), "adjacent: [ %s]", ss.str().c_str());
                        }

                        // 2、把points排序，生成路线
                        int start_index = path_searcher_->get_start_point_index(robot_x, robot_y, points);
                        path_searcher_->process_(points, start_index);

                        auto path = path_searcher_->get_path();
                        path_searcher_->filter_path(points, path);
                        std::stringstream ss;
                        for (size_t i = 0; i < path.size(); i++)
                        {
                                ss << path[i] << " ";
                        }
                        RCLCPP_INFO(get_logger(), "filtered path : [ %s]", ss.str().c_str());

                        // 3、根据遍历的路径顺序，逐段生成path
                        std::vector<garage_utils_pkg::EnhancedPoint> points_ordered;
                        for(size_t i = 0; i < path.size(); i++)
                        {
                                garage_utils_pkg::EnhancedPoint e_point;
                                for (size_t j = 0; j < points.size(); j++)
                                {
                                        if (points[j].index == path[i])
                                        {
                                                e_point = points[j];
                                                RCLCPP_INFO(get_logger(), "order: %d", path[i]);
                                                RCLCPP_INFO(get_logger(), "points => index: %d, coord: (%f, %f)",
                                                        points[j].index, points[j].coord.first, points[j].coord.second);
                                                points_ordered.push_back(e_point);
                                                break;
                                        }
                                } 
                        }     
                        
                        path_generator_->process_(points_ordered, this->polygons_);
                        this->path_ = path_generator_->get_path();

                        // 4、publish path
                        this->path_pub_->publish(this->path_);

                        RCLCPP_INFO(get_logger(), "----- print path's poses -----");
                        for (size_t i = 0; i < this->path_.poses.size(); i++)
                        {
                                RCLCPP_INFO(get_logger(), "poses: (%.1f, %.1f)",
                                        this->path_.poses[i].pose.position.x,
                                        this->path_.poses[i].pose.position.y);
                        }
                                                 
                        
                        if (rclcpp::ok())
                        {
                                result->success = true;
                                result->path = this->path_;
                                result->poses = this->path_.poses;
                                goal_handle->succeed(result);
                                RCLCPP_INFO(get_logger(), "Goal succeed");
                        }
                }
                catch(const std::exception& e)
                {
                        RCLCPP_ERROR(get_logger(), "catch std::exception: %s", e.what());
                        result->success = false;
                        result->reason = e.what();
                        goal_handle->abort(result);
                }
                catch(const std::string& e)
                {
                        RCLCPP_INFO(get_logger(), "catch std::string exception: %s", e.c_str());
                        result->success = false;
                        result->reason = e;
                        goal_handle->abort(result);
                }                          
        }

        double ComputeRightEdgePathActionServer::theta_between_two_edges(Point seg1_pt1, Point seg1_pt2, Point seg2_pt1, Point seg2_pt2)
        {
              // 计算两条线段的方向向量
                const double vec1_x = seg1_pt2.first - seg1_pt1.first;
                const double vec1_y = seg1_pt2.second - seg1_pt1.second;
                const double vec2_x = seg2_pt2.first - seg2_pt1.first;
                const double vec2_y = seg2_pt2.second - seg2_pt1.second;

                // 计算点积
                const double dot = vec1_x * vec2_x + vec1_y * vec2_y;

                // 计算向量模长
                const double norm1 = std::hypot(vec1_x, vec1_y);
                const double norm2 = std::hypot(vec2_x, vec2_y);

                // 处理无效输入（线段退化为点）
                if (norm1 == 0.0 || norm2 == 0.0) {
                        return 0.0; // 默认返回0，实际可根据需求修改
                }

                // 计算余弦值，并限制在[0, 1]范围内
                double cos_theta = std::abs(dot) / (norm1 * norm2);
                // cos_theta = std::clamp(cos_theta, 0.0, 1.0);

                // 计算并返回夹角（弧度值）
                return std::acos(cos_theta);  
        }

        double ComputeRightEdgePathActionServer::point_to_line_distance_smart(double px, double py, double x1, double y1, double x2, double y2)
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

        bool ComputeRightEdgePathActionServer::get_map_robot_tf()
        {
                std::string errMsg;
                std::string refFrame = "map";
                std::string childFrame = "base_link";
                geometry_msgs::msg::TransformStamped transformStamped;

                if (!this->tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
                        tf2::durationFromSec(0.5), &errMsg))
                {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get TF from " 
                                << refFrame << " to " << childFrame << ": " << errMsg);
                        return false;
                } 
                else 
                {
                        try 
                        {
                                transformStamped = this->tf_buffer_->lookupTransform( refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(0.5));
                                tf2::fromMsg(transformStamped.transform, this->map_robot_tf);
                                return true;       
                        } 
                        catch (const tf2::TransformException & e) 
                        {
                                RCLCPP_ERROR_STREAM(
                                this->get_logger(),
                                "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
                                return false;
                        }
                }
        }

        size_t ComputeRightEdgePathActionServer::get_current_polygon_index(tf2::Transform robot_pose, std::vector<geometry_msgs::msg::Polygon> polygons)
        {
                double robot_x, robot_y; 
                robot_x = robot_pose.getOrigin().getX();
                robot_y = robot_pose.getOrigin().getY();

                for (size_t i = 0; i < polygons_.size(); i++)
                {
                        auto polygon = polygons[i];
                        auto points_size = polygon.points.size();
                        std::vector<std::pair<double, double>> polygon_segment_vec;
                        for (size_t j = 0; j < polygon.points.size(); j++)
                        {
                                auto point = polygon.points[j];
                                std::pair<double, double> segment;
                                segment.first = point.x;
                                segment.second = point.y;
                                polygon_segment_vec.push_back(segment);
                        }
                        if (isPointInPolygon(robot_x, robot_y, polygon_segment_vec))
                        {
                                return i;
                        }
                        else
                        {
                                continue;
                        }                 
                }

                double distance_polygon_min = std::numeric_limits<double>::max();
                size_t index = 0;

                for (size_t i = 0; i < polygons_.size(); i++)
                {
                        auto polygon = polygons_[i];
                        double distance_side_min = std::numeric_limits<double>::max();
                        for (size_t j = 0; j < polygon.points.size(); j++)
                        {
                                auto point1 = polygon.points[j];
                                auto point2 = polygon.points[(j+1)%(polygon.points.size())];
                                double distance = point_to_line_distance_smart(robot_x, robot_y, point1.x, point1.y, point2.x, point2.y);
                                if (distance < distance_side_min)
                                {
                                        distance_side_min = distance;
                                }
                        }
                        if (distance_side_min < distance_polygon_min)
                        {
                                distance_polygon_min = distance_side_min;
                                index = i;
                        }
                }
                return index;
        }

        bool ComputeRightEdgePathActionServer::isPointOnSegment(double px, double py, double x1, double y1, double x2, double y2)
        {
                // 判断点是否在线段端点范围内
                if ((px < std::min(x1, x2) - 1e-8) || (px > std::max(x1, x2) + 1e-8) ||
                        (py < std::min(y1, y2) - 1e-8) || (py > std::max(y1, y2) + 1e-8)) 
                        return false;

                // 共线检测‌
                double cross = (x2 - x1)*(py - y1) - (y2 - y1)*(px - x1);
                return fabs(cross) < 1e-8;
        }

        bool ComputeRightEdgePathActionServer::isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon)
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

        bool ComputeRightEdgePathActionServer::goal_checker(std::shared_ptr<const ComputeRightEdgePath::Goal> goal, std::string& reason)
        {
                bool ret = true;

                auto polygons = goal->polygons;
                for (size_t i = 0; i < polygons.size(); i++)
                {
                        auto points = polygons[i].points;
                        if (points.size() != 4)
                        {
                                reason = "Every polygon shall have 4 points.";
                                return false;
                        }

                        for (size_t j = 0; j < points.size() - 1; j++)
                        {
                                for (size_t k = j + 1; k < points.size(); k++)
                                {
                                        auto pt1 = points[j];
                                        auto pt2 = points[k];
                                        if (pt1 == pt2)
                                        {
                                                reason = "Every point of each polycon shall different.";
                                                return false;
                                        }
                                }
                        }

                        for (size_t l = 0; l < points.size(); l++)
                        {
                                auto pt1 = points[l];
                                auto segment_pt1 = points[(l+1)%4];
                                auto segment_pt2 = points[(l+2)%4];
                                if (isPointOnSegment(pt1.x, pt1.y, segment_pt1.x, segment_pt1.y, segment_pt2.x, segment_pt2.y))
                                {
                                        reason = "Any three points cannot be on a straight line";
                                        return false;
                                }
                        }
                }

                return ret;
        }

} // end of namespace
