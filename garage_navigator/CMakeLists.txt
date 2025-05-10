cmake_minimum_required(VERSION 3.5)
project(garage_navigator)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(garage_behavior_tree REQUIRED)
find_package(garage_utils_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name bt_garage_navigator)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name ${executable_name}_core)


set(dependencies
  ament_cmake
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  geometry_msgs
  behaviortree_cpp_v3
  nav2_util
  nav2_core
  garage_utils_msgs
  nav2_behavior_tree
  garage_behavior_tree
  tf2_ros
  visualization_msgs
  nlohmann_json
)

add_library(${library_name} SHARED
  src/bt_garage_navigator.cpp
  src/garage_vehicle_avoidance.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "garage_utils_pkg::BtNavigator")

# test node
add_executable(test_action src/test.cpp)
ament_target_dependencies(test_action
  ${dependencies}
)

# test for /global_costmap
add_executable(test_global_costmap src/sub_global_costmap.cpp)
ament_target_dependencies(test_global_costmap
  ${dependencies}
)

# polygons_markers_pub node
add_executable(publisher_polygons_marker src/pub_polygons_markers.cpp)
ament_target_dependencies(publisher_polygons_marker
  ${dependencies}
)

# nav_through_poses test node
add_executable(test_nav_through_poses src/navigate_through_poses_test.cpp)
ament_target_dependencies(test_nav_through_poses
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} test_action test_nav_through_poses publisher_polygons_marker test_global_costmap
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees launch params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
