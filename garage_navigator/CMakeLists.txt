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

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} test_action
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
