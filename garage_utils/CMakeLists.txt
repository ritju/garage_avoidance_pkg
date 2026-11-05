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
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(capella_ros_msg REQUIRED)
find_package(nav2_util REQUIRED)
set(dependencies 
  rclcpp
  rclcpp_action
  rclcpp_components
  garage_utils_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  capella_ros_msg
  nav2_util
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(generate_model SHARED
  src/generate_model.cpp)
target_include_directories(generate_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(generate_model ${dependencies})

add_library(shortest_path_search SHARED
  src/shortest_path_search.cpp)
target_include_directories(shortest_path_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(shortest_path_search ${dependencies})

add_library(generate_path SHARED
  src/generate_path.cpp src/footprint_check.cpp)
target_include_directories(generate_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(generate_path ${dependencies})

add_library(compute_right_edge_path_server SHARED src/compute_right_edge_path.cpp)
target_include_directories(compute_right_edge_path_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(compute_right_edge_path_server ${dependencies})
target_link_libraries(compute_right_edge_path_server generate_model shortest_path_search generate_path)
rclcpp_components_register_node(
  compute_right_edge_path_server PLUGIN "garage_utils_pkg::ComputeRightEdgePathActionServer" 
  EXECUTABLE compute_right_edge_path_server_node)

install(TARGETS compute_right_edge_path_server generate_model shortest_path_search generate_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
