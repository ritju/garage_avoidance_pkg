#include "garage_utils/compute_right_edge_path.hpp"

namespace garage_utils_pkg
{
        ComputeRightEdgePathActionServer::ComputeRightEdgePathActionServer(const rclcpp::NodeOptions & options)
                : Node("compute_right_edge_path_action_server", options)
        {
                using namespace std::placeholders;

                // init tf2
                this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

                std::vector<geometry_msgs::msg::Polygon>().swap(polygons_);

                auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                auto pub_ops = rclcpp::PublisherOptions();
                pub_ops.callback_group = callback_group;
                path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_right_edge", rclcpp::QoS{1}.best_effort(), pub_ops);

                this->action_server_ = rclcpp_action::create_server<ComputeRightEdgePath>(
                this,
                "compute_right_edge_path_action_server",
                std::bind(&ComputeRightEdgePathActionServer::handle_goal, this, _1, _2),
                std::bind(&ComputeRightEdgePathActionServer::handle_cancel, this, _1),
                std::bind(&ComputeRightEdgePathActionServer::handle_accepted, this, _1));
        }
        
        ComputeRightEdgePathActionServer::~ComputeRightEdgePathActionServer()
        {

        }

        rclcpp_action::GoalResponse ComputeRightEdgePathActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const ComputeRightEdgePath::Goal> goal)
        {
                RCLCPP_INFO(this->get_logger(), "Received goal request with %ld polygons, the pose of car (%f, %f).", 
                        goal->polygons.size(), goal->car_pose.pose.position.x, goal->car_pose.pose.position.y);
                geometry_msgs::msg::Polygon polygon;
                geometry_msgs::msg::Point32 point;
                for (size_t i = 0; i < goal->polygons.size(); i++)
                {
                        std::stringstream ss;
                        ss << "[";
                        polygon = goal->polygons[i];
                        for (size_t j = 0; j < polygon.points.size(); j++)
                        {
                                point = polygon.points[j];
                                ss << "(" <<  point.x << ", " << point.y << ")";
                                if (j != (polygon.points.size() - 1))
                                {
                                        ss << ", ";
                                }
                        }
                        ss << "]";
                        RCLCPP_INFO(get_logger(), "polygon %zd: %s", i, ss.str().c_str());
                }
                
                (void)uuid;

                // 存储polygons
                auto feedback = std::make_shared<ComputeRightEdgePath::Feedback>();
                auto polygons_msg = goal.get()->polygons;
                if(polygons_ == polygons_msg)
                {
                        // polygons not updated
                }
                else
                {
                        polygons_ = polygons_msg;
                        polygons_pair_vec.clear();
                        for (size_t i = 0; i < polygons_.size(); i++)
                        {
                                std::pair<bool, geometry_msgs::msg::Polygon> pair;
                                pair.first = false;
                                pair.second = polygons_[i];
                                polygons_pair_vec.push_back(pair);
                        }
                }
                
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse ComputeRightEdgePathActionServer::handle_cancel(const std::shared_ptr<GoalHandleComputeRightEdgePath> goal_handle)
        {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                // (void)goal_handle;
                
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
                RCLCPP_INFO(get_logger(), "Begin to compute path following right edge.");
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<ComputeRightEdgePath::Feedback>();

                get_map_robot_tf();

                double robot_x, robot_y, car_x, car_y, angle_car_to_robot;
                robot_x = map_robot_tf.getOrigin().getX();
                robot_y = map_robot_tf.getOrigin().getY();
                car_x = goal->car_pose.pose.position.x;
                car_y = goal->car_pose.pose.position.y;
                angle_car_to_robot = std::atan2(robot_y - car_y, robot_x - car_x);

                size_t current_index = get_current_polygon_index(map_robot_tf, polygons_);
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

} // end of namespace
