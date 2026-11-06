cmake_minimum_required(VERSION 3.8)
project(warrior_mti680g_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# The workspace targets Gazebo Fortress (ROS 2 Humble / RoboStack).
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_executable(mti680g_sim src/mti680g_sim.cpp)
target_compile_features(mti680g_sim PUBLIC cxx_std_17)
target_compile_options(mti680g_sim PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(mti680g_sim PRIVATE include)
ament_target_dependencies(mti680g_sim rclcpp sensor_msgs geometry_msgs)
target_link_libraries(mti680g_sim ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)
install(TARGETS mti680g_sim DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(mti680g_model_test test/mti680g_model_test.cpp)
  target_compile_features(mti680g_model_test PUBLIC cxx_std_17)
  target_include_directories(mti680g_model_test PRIVATE include)
  add_test(NAME mti680g_model COMMAND mti680g_model_test)
endif()

ament_package()
