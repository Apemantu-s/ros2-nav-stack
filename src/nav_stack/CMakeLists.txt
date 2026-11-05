cmake_minimum_required(VERSION 3.8)
project(nav_stack)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(ros_deps rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros)

add_library(nav_core
  src/ekf.cpp
  src/grid.cpp
  src/costmap.cpp
  src/astar.cpp
  src/rrt_star.cpp)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(nav_core Eigen3::Eigen)

foreach(node wheel_odom_node ekf_node mapper_node)
  add_executable(${node} nodes/${node}.cpp)
  target_link_libraries(${node} nav_core)
  ament_target_dependencies(${node} ${ros_deps})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nav_core
    test/test_ekf.cpp
    test/test_mapper.cpp
    test/test_planners.cpp)
  target_link_libraries(test_nav_core nav_core)
endif()

ament_package()