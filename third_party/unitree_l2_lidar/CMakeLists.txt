cmake_minimum_required(VERSION 3.16)
project(unitree_l2_lidar NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
