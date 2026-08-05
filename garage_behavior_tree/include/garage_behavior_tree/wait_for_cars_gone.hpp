#ifndef GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_
#define GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "capella_ros_msg/msg/car_detect_array.hpp"

namespace garage_utils_pkg
{

/**
 * @brief 避车结束后的动态等待节点。
 *
 * 贴边避车（WeltModelNode）完成后持续观察 /car_information：
 *  - 等待期间收到含车的消息 → 重置 idle_timeout 计时（还有车，继续等）
 *  - 连续 idle_timeout 秒没有车 → 返回 SUCCESS（车流通过，避车等待结束）
 *  - 可选 max_wait 总超时（>0 才启用），防止车流一直很大导致任务卡死
 */
class WaitForCarsGone : public BT::ActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name XML 标签名
   * @param conf BT 节点配置
   */
  WaitForCarsGone(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  WaitForCarsGone() = delete;

  /**
   * @brief tick 主逻辑
   * @return RUNNING 继续等待；SUCCESS 等待结束
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 行为树取消/中断时调用，重置内部状态
   */
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("idle_timeout", 15.0,
        "连续多少秒没有检测到车则结束避车等待"),
      BT::InputPort<double>("max_wait", -1.0,
        "最大等待总时长（秒），<=0 表示不限（默认不启用）"),
      BT::InputPort<std::string>("topic_name", "/car_information",
        "车辆信息话题"),
    };
  }

private:
  /**
   * @brief /car_information 订阅回调：收到含车的消息则重置计时
   */
  void carInformationCallback(capella_ros_msg::msg::CarDetectArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_info_sub_;

  std::mutex mutex_;          // 保护 last_car_time_
  double idle_timeout_ = 15.0;
  double max_wait_ = -1.0;
  std::string topic_name_ = "/car_information";

  bool started_ = false;
  rclcpp::Time start_time_;    // 开始等待的时刻
  rclcpp::Time last_car_time_; // 最近一次收到车的时间
};

}  // namespace garage_utils_pkg

#endif  // GARAGE_BEHAVIOR_TREE__WAIT_FOR_CARS_GONE_HPP_
