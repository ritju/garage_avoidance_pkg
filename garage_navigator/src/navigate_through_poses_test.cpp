#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "capella_ros_msg/msg/car_detect_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "garage_utils_msgs/action/garage_vehicle_avoidance.hpp"
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavigateThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class NavThroughPosesClient : public rclcpp::Node {
public:
  NavThroughPosesClient(int total_executions) 
  : Node("nav_through_poses_client"), total_executions_(total_executions) {
    client_nav_throuth_poses_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
    initialize_poses();

    client_garage_ = rclcpp_action::create_client<garage_utils_msgs::action::GarageVehicleAvoidance>(
                this, "/garage_vehicle_avoidance");

    auto cb1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_op1 = rclcpp::SubscriptionOptions();
    sub_op1.callback_group = cb1;
    is_car_passable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_is_car_passable_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&NavThroughPosesClient::is_car_passable_callback, this, std::placeholders::_1), sub_op1);

    auto cb2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_op2 = rclcpp::SubscriptionOptions();
    sub_op2.callback_group = cb2;
    car_information_sub_ = this->create_subscription<capella_ros_msg::msg::CarDetectArray>(
        topic_car_information_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&NavThroughPosesClient::car_information_callback, this, std::placeholders::_1), sub_op2);
  }

  void NavThroughPosesClient::is_car_passable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_car_passable_ = msg->data;
  }

  void NavThroughPosesClient::car_information_callback(const capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
  {
    car_informations_ = *msg;
  }

  void run() {
    if (!client_nav_throuth_poses_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    for (int i = 0; i < total_executions_; ++i) {
      RCLCPP_INFO(this->get_logger(), "========== Starting execution %d/%d ==========", 
                i+1, total_executions_);
      if (!execute_navigation_cycle()) {
        RCLCPP_ERROR(this->get_logger(), "Navigation cycle %d failed", i+1);
      }
      RCLCPP_INFO(this->get_logger(), "========== Completed execution %d/%d ==========\n", 
                i+1, total_executions_);
    }
  }

  std::vector<std::vector<double>> parse_json_array(const std::string& json_str) 
  {
    auto json = nlohmann::json::parse(json_str);
    std::vector<std::vector<double>> result;

    for (const auto& inner_array : json) {
        std::vector<double> vec;
        for (auto val : inner_array) vec.push_back(val.get<double>());
        result.push_back(vec);
    }
    return result;
  }

private:
  void initialize_poses() {
    // 初始化多个导航点
    poses_.clear();
    poses_.push_back(create_pose(-7.47, -7.59, M_PI));
    poses_.push_back(create_pose(4.6, -8.58, 0));
    poses_.push_back(create_pose(-3.17, -14.66, M_PI_2));
    poses_.push_back(create_pose(4.6, -8.58, 0));
  }

  geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;
  }

  bool execute_navigation_cycle() {
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = poses_;
    goal_msg.behavior_tree = "/workspaces/capella_coverage_server/install/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml";

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = 
      [this](const NavigateThroughPosesGoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Navigation completed successfully!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Navigation failed with code: %d", result.code);
        }
      };

    // 发送目标
    auto future_goal_handle = client_nav_throuth_poses_->async_send_goal(goal_msg, send_goal_options);

    // 等待目标接受
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
      nav_through_poses_action_executing = false;
      return false;
    }
    else
    {
      nav_through_poses_action_executing = true;
    }

    navigate_through_poses_goal_handle = future_goal_handle.get();
    if (!navigate_through_poses_goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      nav_through_poses_action_executing = false;
      return false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal was accepted by server");
      nav_through_poses_action_executing = true;
      if (!timer_check_is_car_passable_)
      {
        timer_check_is_car_passable_ = this->create_wall_timer(
          std::chrono::milliseconds(10), std::bind(&NavThroughPosesClient::timer_callback, this, std::placeholders::_1));
      }
    }

    // 等待执行结果
    auto result_future = client_nav_throuth_poses_->async_get_result(navigate_through_poses_goal_handle);
    RCLCPP_INFO(this->get_logger(), "Waiting for navigation to complete...");
    
    auto status = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future);
    
    nav_through_poses_action_executing = false;
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result");
      return false;
    }

    auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void garage_goal_response_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr& goal_handle)
  {
    std::cout << "garage_vehicle goal response callback" << std::endl;
    goal_handle_ = goal_handle;
    if (!goal_handle_) 
    {
      std::cout << "garage_vehicle goal was rejected by server" << std::endl;
      garage_action_executing = true;
    } 
    else 
    {
      std::cout << "garage_vehicle goal accepted by server, waiting for result" << std::endl;
      garage_action_executing = false;
    }
  }

  void garage_feedback_callback(rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr goal_handle, const std::shared_ptr<const garage_utils_msgs::action::GarageVehicleAvoidance::Feedback> feedback)
  {

  }

  void garage_result_callback(const rclcpp_action::ClientGoalHandle<garage_utils_msgs::action::GarageVehicleAvoidance>::WrappedResult& result)
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      std::cout << "Goal was succeed." << std::endl;
      
      break;

      case rclcpp_action::ResultCode::ABORTED:
      std::cout << "Goal was aborted" << std::endl;
      return;

      case rclcpp_action::ResultCode::CANCELED:
      std::cout << "Goal was canceled" << std::endl;
      return;

      default:
      std::cout << "Unknown result code" << std::endl;
      return;
    }
    garage_action_executing = false;
    std::cout << "reason: " << result.result->reason << std::endl;
  }

  void timer_callback()
  {
    if (!is_car_passable_ && !nav_through_poses_action_executing)
    {
      timer_check_is_car_passable_.reset();
      client_nav_throuth_poses_->async_cancel_goal(navigate_through_poses_goal_handle);

      auto goal_msg = garage_utils_msgs::action::GarageVehicleAvoidance::Goal();
      goal_msg.cars_information = car_informations_;
      goal_msg.polygons = polygons_;
      RCLCPP_INFO(node->get_logger(), "Sending goal for garage avoidance ...");
      auto send_goal_options = rclcpp_action::Client<garage_utils_msgs::action::GarageVehicleAvoidance>::SendGoalOptions();
      send_goal_options.goal_response_callback = garage_goal_response_callback;
      send_goal_options.result_callback = garage_result_callback;
      send_goal_options.feedback_callback = garage_feedback_callback;
      client_garage_->async_send_goal(goal_msg);
    }
  }

  void init_params()
  {
    this->declare_parameter<std::string>("topic_is_car_passable");
    this->declare_parameter<std::string>("topic_car_information");
    this->declare_parameter<std::string>("through_poses");
    this->declare_parameter<std::string>("polygons");

    this->get_parameter_or<std::string>()
  }

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_nav_throuth_poses_;  

  rclcpp_action::Client<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr client_garage_;

  // params
  std::vector<geometry_msgs::msg::PoseStamped> navigation_through_poses_;
  int total_executions_;
  std::string topic_is_car_passable_;
  std::string topic_car_information_;
  std::vector<std::string> polygons_vec_;

  void init_params();

  // timer for check is_car_passable_
  rclcpp::TimerBase::SharedPtr timer_check_is_car_passable_;
  void timer_callback();

  // subs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_car_passable_sub_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_information_sub_;

  // subs callback
  void is_car_passable_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void car_information_callback(const capella_ros_msg::msg::CarDetectArray::SharedPtr msg);

  bool is_car_passable_{true};
  capella_ros_msg::msg::CarDetectArray car_informations_;
  std::vector<geometry_msgs::msg::Polygon> polygons_;
  bool garage_action_executing{false};
  bool nav_through_poses_action_executing{false};

  NavigateThroughPosesGoalHandle::SharedPtr navigate_through_poses_goal_handle;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  // 设置执行次数
  const int TOTAL_EXECUTIONS = 3000;
  
  auto node = std::make_shared<NavThroughPosesClient>(TOTAL_EXECUTIONS);
  node->run();
  
  rclcpp::shutdown();
  return 0;
}