#include "garage_behavior_tree/is_avoid_direction_has_car.hpp"


namespace garage_utils_pkg
{


IsAvoidDirectionHasCar::IsAvoidDirectionHasCar(
    const std::string & name,
    const BT::NodeConfiguration & conf)
:
BT::ConditionNode(name, conf),
car_pose_valid_(false),
avoid_side_has_car_(true)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<tf2_ros::Buffer::SharedPtr>("tf_buffer");

    low_speed_timeout_ = node_->declare_parameter("avoid_low_speed_timeout", 8.0);
    high_speed_timeout_ = node_->declare_parameter("avoid_high_speed_timeout", 3.0);
    speed_threshold_ = node_->declare_parameter("avoid_speed_threshold", 0.3);
    side_angle_threshold_ = node_->declare_parameter("avoid_side_angle", 90.0 * M_PI / 180.0);

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    car_sub_ = node_->create_subscription<capella_ros_msg::msg::CarDetectArray>
    (
        "/car_information_all",
        rclcpp::QoS(1),
        std::bind(
            &IsAvoidDirectionHasCar::carInformationCallback,
            this,
            std::placeholders::_1),
        sub_option
    );

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>
    (
        "/odom",
        10,
        std::bind(
            &IsAvoidDirectionHasCar::odomCallback,
            this,
            std::placeholders::_1),
        sub_option
    );

    last_car_time_ = node_->now();
}


void IsAvoidDirectionHasCar::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (car_pose_valid_)
    {
        speed_sum_ += std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        speed_count_++;
    }
}


void IsAvoidDirectionHasCar::carInformationCallback(
    const capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
{
    // car_pose_ 还未由 tick 初始化，不做判断
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!car_pose_valid_ || !avoid_side_has_car_)
    {
        return;
    }

    if(!nav2_util::getCurrentPose(robot_pose_, *tf_buffer_, "map", "base_link", 0.1))
    {
        RCLCPP_WARN(node_->get_logger(), "[IsAvoidDirectionHasCar] get robot pose failed");
        return;
    }

    double vx = car_pose_.pose.position.x - robot_pose_.pose.position.x;
    double vy = car_pose_.pose.position.y - robot_pose_.pose.position.y;

    double min_dist = std::numeric_limits<double>::max();
    bool found = false;
    double nearest_pose_angle = 0.0;
    geometry_msgs::msg::PoseStamped nearest_pose;

    for(auto &car : msg->results)
    {
        double vcx = car.pose.pose.position.x - robot_pose_.pose.position.x;
        double vcy = car.pose.pose.position.y - robot_pose_.pose.position.y;

        // 点积
        double dot = vx * vcx + vy * vcy;

        // 模长
        double norm1 = std::sqrt(vx * vx + vy * vy);
        double norm2 = std::sqrt(vcx * vcx + vcy * vcy);

        if(norm1 < 1e-6 || norm2 < 1e-6)
        {
            continue;
        }

        double cos_theta = dot / (norm1 * norm2);
        cos_theta = std::clamp(cos_theta, -1.0, 1.0);
        double angle = std::acos(cos_theta);
        // 与动态目标车方向的夹角
        if (angle >= side_angle_threshold_)
        {
            continue;
        }

        // 考虑车一直跟着，机器人多次转弯还跟着 这儿就有矛盾
        // 与基准方向 a（tick初始化时机器人→car_pose）的夹角
        // double cos_base = (vcx * base_dir_x_ + vcy * base_dir_y_) / norm2;
        // cos_base = std::clamp(cos_base, -1.0, 1.0);
        // double angle_base = std::acos(cos_base);

        // if (angle_base >= side_angle_threshold_)
        // {
        //     continue;
        // }

        // 夹角均满足，进入距离比较
        double dist = std::hypot(vcx, vcy);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_pose = car.pose;
            found = true;
            nearest_pose_angle = angle;
        }
    }

    if(found)
    {
        car_pose_ = nearest_pose;
        avoid_side_has_car_=true;
        last_car_time_ = node_->now();
        speed_sum_   = 0.0;
        speed_count_ = 0;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[IsAvoidDirectionHasCar] avoid side car detected (nearest, same direction, angle:%.1f rad)", nearest_pose_angle);
    }
    else
    {
        double avg_speed = (speed_count_ > 0) ? (speed_sum_ / speed_count_) : 0.0;
        double timeout = avg_speed > speed_threshold_ ? high_speed_timeout_ : low_speed_timeout_;

        double dt = (node_->now() - last_car_time_).seconds();
        if(dt > timeout) {
            avoid_side_has_car_=false;
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[IsAvoidDirectionHasCar] avoid side clear (no same-direction car for %.1f s，speed: %.1f)", dt, avg_speed);
        }
    }
}

BT::NodeStatus IsAvoidDirectionHasCar::tick()
{
    geometry_msgs::msg::PoseStamped pose;

    if(!getInput("car_pose", pose))
    {
        RCLCPP_WARN(node_->get_logger(), "[IsAvoidDirectionHasCar] car_pose missing");
        return BT::NodeStatus::SUCCESS;
    }

    if (std::hypot(
            pose.pose.position.x - original_car_pose_.pose.position.x,
            pose.pose.position.y - original_car_pose_.pose.position.y
        ) > 1e-6)
    {
        // 获取机器人当前位姿，用于记录基准方向
        // geometry_msgs::msg::PoseStamped init_robot_pose;
        // if(!nav2_util::getCurrentPose(init_robot_pose, *tf_buffer_, "map", "base_link", 0.1))
        // {
        //     RCLCPP_WARN(node_->get_logger(), "get robot pose failed when initializing base direction");
        //     return BT::NodeStatus::SUCCESS;
        // }

        // double ax = pose.pose.position.x - init_robot_pose.pose.position.x;
        // double ay = pose.pose.position.y - init_robot_pose.pose.position.y;
        // double norm = std::hypot(ax, ay);
        // if(norm > 1e-6)
        // {
        //     base_dir_x_ = ax / norm;
        //     base_dir_y_ = ay / norm;
        // }
        // else
        // {
        //     RCLCPP_WARN(node_->get_logger(), "robot is too close to car_pose, base direction undefined");
        //     return BT::NodeStatus::SUCCESS;
        // }

        RCLCPP_INFO(node_->get_logger(), "[IsAvoidDirectionHasCar] new car_pose");

        original_car_pose_ = pose;
        car_pose_ = pose;
        car_pose_valid_ = true;
        avoid_side_has_car_ = true;
        speed_sum_ = 0.0;
        speed_count_ = 0;
        last_car_time_ = node_->now();   // 重置计时，从现在开始等待 callback 确认
    }

    callback_group_executor_.spin_some();

    if (avoid_side_has_car_)
    {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[IsAvoidDirectionHasCar] has car");
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[IsAvoidDirectionHasCar] not has car");

    // 无车：重置 car_pose_ 为"未设置"，等下次 tick 重新赋值后再循环
    car_pose_valid_  = false;
    return BT::NodeStatus::FAILURE;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<garage_utils_pkg::IsAvoidDirectionHasCar>("IsAvoidDirectionHasCar");
}
