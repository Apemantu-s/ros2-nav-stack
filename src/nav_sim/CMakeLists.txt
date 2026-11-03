cmake_minimum_required(VERSION 3.8)
project(nav_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds config launch DESTINATION share/${PROJECT_NAME})

ament_package()