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

add_executable(wheel_odom_node nodes/wheel_odom_node.cpp)
ament_target_dependencies(wheel_odom_node rclcpp nav_msgs sensor_msgs geometry_msgs)
install(TARGETS wheel_odom_node DESTINATION lib/${PROJECT_NAME})

ament_package()