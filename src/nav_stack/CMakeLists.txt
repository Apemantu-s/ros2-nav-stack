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

add_executable(wheel_odom_node nodes/wheel_odom_node.cpp)
ament_target_dependencies(wheel_odom_node rclcpp nav_msgs sensor_msgs geometry_msgs)
install(TARGETS wheel_odom_node DESTINATION lib/${PROJECT_NAME})

add_executable(ekf_node nodes/ekf_node.cpp)
target_link_libraries(ekf_node nav_core)
ament_target_dependencies(ekf_node rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros)
install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})

find_package(Eigen3 REQUIRED)

add_library(nav_core 
  src/ekf.cpp
  src/grid.cpp)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(nav_core Eigen3::Eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nav_core 
    test/test_mapper.cpp
    test/test_ekf.cpp)
  target_link_libraries(test_nav_core nav_core)
endif()

ament_package()