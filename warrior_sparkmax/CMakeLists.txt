cmake_minimum_required(VERSION 3.8)
project(warrior_sparkmax)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(warrior_msgs REQUIRED)

add_executable(spark_max_driver src/spark_max_driver.cpp)

target_include_directories(spark_max_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(spark_max_driver PUBLIC cxx_std_17)

ament_target_dependencies(spark_max_driver rclcpp warrior_msgs)

install(TARGETS spark_max_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL)

ament_package()
