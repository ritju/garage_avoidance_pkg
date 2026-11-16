cmake_minimum_required(VERSION 3.8)
project(garage_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(garage_utils_msgs REQUIRED)
find_package(capella_ros_msg REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  ament_cmake
  nav2_common
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav2_util
  garage_utils_msgs
  capella_ros_msg
)

# compute_right_edge_path_action
add_library(compute_right_edge_path_action SHARED src/compute_right_edge_path_action.cpp)
list(APPEND plugin_libs compute_right_edge_path_action)


# find_car_avoidance_point_action
add_library(find_car_avoidance_point_action SHARED src/find_car_avoidance_point_action.cpp)
list(APPEND plugin_libs find_car_avoidance_point_action)

add_library(welt_model_node SHARED src/welt_model_node.cpp)
list(APPEND plugin_libs welt_model_node)

add_library(is_car_moving SHARED src/is_car_moving.cpp)
list(APPEND plugin_libs is_car_moving)

# wait_for_cars_gone
add_library(wait_for_cars_gone SHARED src/wait_for_cars_gone.cpp)
list(APPEND plugin_libs wait_for_cars_gone)

add_library(stop_when_cars_gone SHARED src/stop_when_cars_gone.cpp)
list(APPEND plugin_libs stop_when_cars_gone)

add_library(wait_for_far_side SHARED src/wait_for_far_side.cpp)
list(APPEND plugin_libs wait_for_far_side)

add_library(far_side_guard SHARED src/far_side_guard.cpp)
list(APPEND plugin_libs far_side_guard)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}                
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_behavior_tree REQUIRED)
  ament_add_gtest(test_stop_when_cars_gone test/test_stop_when_cars_gone.cpp)
  ament_target_dependencies(test_stop_when_cars_gone ${dependencies})
  target_link_libraries(test_stop_when_cars_gone
    stop_when_cars_gone far_side_guard wait_for_cars_gone find_car_avoidance_point_action)
  target_compile_definitions(test_stop_when_cars_gone PRIVATE
    BT_XML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../garage_navigator/behavior_trees"
    ROS_BT_LIB_DIR="${nav2_behavior_tree_DIR}/../../../lib")
  set_tests_properties(test_stop_when_cars_gone PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=197;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()

