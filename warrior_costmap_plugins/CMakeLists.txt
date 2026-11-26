cmake_minimum_required(VERSION 3.8)
project(warrior_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(deps nav2_costmap_2d pluginlib rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

add_library(${PROJECT_NAME} SHARED src/lane_grid.cpp src/lane_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${deps})

pluginlib_export_plugin_description_file(nav2_costmap_2d lane_layer_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES lane_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Use the prebuilt GTest from the Pixi environment: the gtest_vendor
  # sources that ament_add_gtest compiles do not build with this GCC
  # (missing <cstdint> in gtest-death-test.cc).
  find_package(ament_cmake_test REQUIRED)
  find_package(GTest REQUIRED)
  add_executable(test_lane_grid test/test_lane_grid.cpp)
  target_link_libraries(test_lane_grid ${PROJECT_NAME} GTest::gtest_main)
  ament_add_test(test_lane_grid
    COMMAND $<TARGET_FILE:test_lane_grid>
      --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_lane_grid.gtest.xml
    RESULT_FILE ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_lane_grid.gtest.xml)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${deps})
ament_package()
