#pragma once

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "capella_ros_msg/msg/car_detect_array.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"


namespace garage_utils_pkg
{


class IsAvoidDirectionHasCar : public BT::ConditionNode
{

public:

    IsAvoidDirectionHasCar(
        const std::string & name,
        const BT::NodeConfiguration & conf);


    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<geometry_msgs::msg::PoseStamped>(
                "car_pose")
        };
    }


    BT::NodeStatus tick() override;


private:


    void carInformationCallback(
        const capella_ros_msg::msg::CarDetectArray::SharedPtr msg);

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // subscriber
    rclcpp::Subscription<
        capella_ros_msg::msg::CarDetectArray
    >::SharedPtr car_sub_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;

    // tf
    tf2_ros::Buffer::SharedPtr tf_buffer_;

    std::mutex state_mutex_;
    
    // odom速度
    double speed_sum_   {0.0};
    size_t speed_count_ {0};

    geometry_msgs::msg::PoseStamped original_car_pose_;
    // 当前避车目标
    geometry_msgs::msg::PoseStamped car_pose_;
    // 当前车pose是否有效
    bool car_pose_valid_;
    // tick初始化时记录的机器人→car_pose基准方向向量（单位化）
    double base_dir_x_ {0.0};
    double base_dir_y_ {0.0};

    // 当前机器人pose
    geometry_msgs::msg::PoseStamped robot_pose_;

    // 最近一次避让侧车辆时间
    rclcpp::Time last_car_time_;

    // 避让侧是否有车
    bool avoid_side_has_car_;

    // 参数
    double low_speed_timeout_;
    double high_speed_timeout_;
    double speed_threshold_;

    // 车辆距离
    double side_angle_threshold_;
};

}