cmake_minimum_required(VERSION 3.8)
project(garage_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(garage_utils_msgs REQUIRED)
set(dependencies 
  rclcpp
  rclcpp_action
  rclcpp_components
  garage_utils_msgs
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(compute_right_edge_path_server SHARED
  src/compute_right_edge_path.cpp)
target_include_directories(compute_right_edge_path_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(compute_right_edge_path_server ${dependencies})
rclcpp_components_register_node(
  compute_right_edge_path_server PLUGIN "garage_utils_pkg::ComputeRightEdgePathActionServer" 
  EXECUTABLE compute_right_edge_path_server_node)
install(TARGETS
  compute_right_edge_path_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
