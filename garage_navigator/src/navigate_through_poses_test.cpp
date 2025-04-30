#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "capella_ros_msg/msg/car_detect_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "garage_utils_msgs/action/garage_vehicle_avoidance.hpp"
#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/polygon.hpp"
#include "garage_utils_msgs/msg/polygons.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavigateThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class NavThroughPosesClient : public rclcpp::Node {
public:
  NavThroughPosesClient(int total_executions) 
  : Node("nav_through_poses_client"), total_executions_(total_executions) 
  {
    polygons_pub_ = this->create_publisher<garage_utils_msgs::msg::Polygons>("garage_polygons", rclcpp::QoS(1).reliable().transient_local());
    polygons_visualization_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rects_visualization", rclcpp::QoS(5).reliable().transient_local());
    init_params();
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

  void is_car_passable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_car_passable_ = msg->data;
  }

  void car_information_callback(const capella_ros_msg::msg::CarDetectArray::SharedPtr msg)
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

private:
  void initialize_poses() {
    // 初始化多个导航点
    navigation_through_poses_.clear();
    auto navigation_through_poses_tmp = parse_json_array(nav_through_poses_);
    for (size_t i = 0; i < navigation_through_poses_tmp.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = navigation_through_poses_tmp[i][0];
      pose.pose.position.y = navigation_through_poses_tmp[i][1];
      navigation_through_poses_.push_back(pose);
    }
    // poses_.push_back(create_pose(-7.47, -7.59, M_PI));
    // poses_.push_back(create_pose(4.6, -8.58, 0));
    // poses_.push_back(create_pose(-3.17, -14.66, M_PI_2));
    // poses_.push_back(create_pose(4.6, -8.58, 0));
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

    while(garage_action_executing)
    {
      RCLCPP_INFO(get_logger(), "Waiting for garage vehicle avoidance action to be completed");
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = navigation_through_poses_;
    goal_msg.behavior_tree = nav_through_poses_bt_tree_;

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr& goal_handle){
        navigate_through_poses_goal_handle = goal_handle;
      };

    send_goal_options.result_callback = 
      [this](const NavigateThroughPosesGoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Navigation completed successfully!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Navigation failed with code: %d", (int)result.code);
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
          std::chrono::milliseconds(10), std::bind(&NavThroughPosesClient::timer_callback, this));
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
    (void) goal_handle;
    std::cout << "garage_vehicle goal response callback" << std::endl;    
    if (!navigate_through_poses_goal_handle) 
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
    (void) goal_handle;
    (void) feedback;
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
      RCLCPP_INFO(get_logger(), "Sending goal for garage avoidance ...");
      auto send_goal_options = rclcpp_action::Client<garage_utils_msgs::action::GarageVehicleAvoidance>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&NavThroughPosesClient::garage_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&NavThroughPosesClient::garage_result_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&NavThroughPosesClient::garage_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      client_garage_->async_send_goal(goal_msg);
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

  void init_params()
  {
    RCLCPP_INFO(get_logger(), "get params.");
    this->declare_parameter<int>("execution_count", 500);
    this->declare_parameter<std::string>("topic_is_car_passable", std::string());
    this->declare_parameter<std::string>("topic_car_information", std::string());
    this->declare_parameter<std::string>("nav_through_poses", std::string());
    this->declare_parameter<std::vector<std::string>>("polygons", std::vector<std::string>());
    this->declare_parameter<std::string>("nav_through_poses_bt_tree", std::string());

    this->get_parameter<int>("execution_count", total_executions_);
    this->get_parameter<std::string>("topic_is_car_passable", topic_is_car_passable_);
    this->get_parameter<std::string>("topic_car_information", topic_car_information_);
    this->get_parameter<std::string>("nav_through_poses", nav_through_poses_);
    this->get_parameter<std::vector<std::string>>("polygons", polygons_vec_);
    this->get_parameter<std::string>("nav_through_poses_bt_tree", nav_through_poses_bt_tree_);

    RCLCPP_INFO(get_logger(), "execution_count: %d", total_executions_);
    RCLCPP_INFO(get_logger(), "topic_is_car_passable: %s", topic_is_car_passable_.c_str());
    RCLCPP_INFO(get_logger(), "topic_car_information: %s", topic_car_information_.c_str());
    RCLCPP_INFO(get_logger(), "nav_through_poses: %s", nav_through_poses_.c_str());
    for(size_t i = 0; i < polygons_vec_.size(); i++)
    {
      RCLCPP_INFO(get_logger(), "polygons: %s", polygons_vec_[i].c_str());
    }
    RCLCPP_INFO(get_logger(), "nav_through_poses_bt_tree: %s", nav_through_poses_bt_tree_.c_str());

    std::vector<std::vector<std::vector<double>>> rects;    
    for (size_t i = 0; i < polygons_vec_.size(); i++)
    {
      auto polygon_tmp = parse_json_array(polygons_vec_[i]);
      rects.push_back(polygon_tmp);
    }

    // pub polygons_visualization msg
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.ns = "test";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    msg.action = 0;
    msg.scale.x = 0.1;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 1.0;
    for (size_t i = 0; i < rects.size(); i++)
    {                
            for (int j = 0; j < 4; j++)
            {
                    geometry_msgs::msg::Point p_start;
                    p_start.x = rects[i][j][0];
                    p_start.y = rects[i][j][1];
                    p_start.z = 0.0;
                    msg.points.push_back(p_start);
                    geometry_msgs::msg::Point p_end;
                    p_end.x = rects[i][(j+1)%4][0];
                    p_end.y = rects[i][(j+1)%4][1];
                    p_end.z = 0.0;
                    msg.points.push_back(p_end);
            }
    }
    
    polygons_visualization_pub_->publish(msg);

    // pub polygons msg
    garage_utils_msgs::msg::Polygons polygons_msg;
    for (size_t i = 0; i < rects.size(); i++)
    {
      geometry_msgs::msg::Polygon polygon;
      for (int j = 0; j < 4; j++)
      {
              geometry_msgs::msg::Point32 point;
              point.x = rects[i][j][0];
              point.y = rects[i][j][1];
              polygon.points.push_back(point);
      }
      polygons_.push_back(polygon);
      polygons_msg.polygons.push_back(polygon);
    }

    // publish polygons msg
    RCLCPP_INFO(get_logger(), "publish garage_polygons topic one time ...");
    polygons_pub_->publish(polygons_msg);
  }

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_nav_throuth_poses_;  

  rclcpp_action::Client<garage_utils_msgs::action::GarageVehicleAvoidance>::SharedPtr client_garage_;

  std::vector<geometry_msgs::msg::PoseStamped> navigation_through_poses_;
  std::vector<geometry_msgs::msg::Polygon> polygons_;
  // params
  int total_executions_;
  std::string topic_is_car_passable_;
  std::string topic_car_information_;
  std::string nav_through_poses_;
  std::vector<std::string> polygons_vec_;
  std::string nav_through_poses_bt_tree_;

  // timer for check is_car_passable_
  rclcpp::TimerBase::SharedPtr timer_check_is_car_passable_;

  // subs
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_car_passable_sub_;
  rclcpp::Subscription<capella_ros_msg::msg::CarDetectArray>::SharedPtr car_information_sub_;

  // pubs
  rclcpp::Publisher<garage_utils_msgs::msg::Polygons>::SharedPtr polygons_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygons_visualization_pub_;

  bool is_car_passable_{true};
  capella_ros_msg::msg::CarDetectArray car_informations_;
  bool garage_action_executing{false};
  bool nav_through_poses_action_executing{false};

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_through_poses_goal_handle;
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