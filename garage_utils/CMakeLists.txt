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
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
set(dependencies 
  rclcpp
  rclcpp_action
  rclcpp_components
  garage_utils_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
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

add_executable(test_generate_model src/test_generate_model.cpp)
target_include_directories(test_generate_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_generate_model generate_model shortest_path_search)
ament_target_dependencies(test_generate_model ${dependencies})

add_executable(test_shortest_path_search src/test_shortest_path_search.cpp)
target_include_directories(test_shortest_path_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_shortest_path_search shortest_path_search)
ament_target_dependencies(test_shortest_path_search ${dependencies})

install(TARGETS
  compute_right_edge_path_server generate_model test_generate_model shortest_path_search test_shortest_path_search
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS test_generate_model test_shortest_path_search
DESTINATION lib/${PROJECT_NAME})

ament_package()
