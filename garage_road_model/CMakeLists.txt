cmake_minimum_required(VERSION 3.8)
project(garage_road_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(garage_utils_msgs QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoadSample.msg"
  "msg/SideBoundary.msg"
  "msg/RoadModel.msg"
  "msg/RoadModelArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(road_model_core SHARED
  src/geometry.cpp
  src/polygon_processor.cpp
  src/curve_fit.cpp
)
target_include_directories(road_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(road_model_core PUBLIC Eigen3::Eigen)

if(garage_utils_msgs_FOUND)
  add_executable(road_model_node
    src/main.cpp
    src/road_model_node.cpp
    src/visualization.cpp
  )
  target_link_libraries(road_model_node road_model_core)
  ament_target_dependencies(road_model_node
    rclcpp
    geometry_msgs
    visualization_msgs
    garage_utils_msgs
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(road_model_node "${cpp_typesupport_target}")
  install(TARGETS road_model_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "未找到 garage_utils_msgs，本次只编译算法库和测试")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_road_model test/test_road_model.cpp)
  target_link_libraries(test_road_model road_model_core)
  if(garage_utils_msgs_FOUND)
    ament_add_gtest(test_node_io test/test_node_io.cpp)
    ament_target_dependencies(test_node_io rclcpp geometry_msgs garage_utils_msgs)
    target_link_libraries(test_node_io "${cpp_typesupport_target}")
    set_tests_properties(test_node_io PROPERTIES
      ENVIRONMENT "ROAD_MODEL_NODE=$<TARGET_FILE:road_model_node>")
  endif()
endif()

install(TARGETS road_model_core
  EXPORT export_road_model_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_road_model_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
