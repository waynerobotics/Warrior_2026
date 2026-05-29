cmake_minimum_required(VERSION 3.16)
project(insta360_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio)

add_executable(insta360_node src/insta360_node.cpp)

target_include_directories(insta360_node PRIVATE ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(insta360_node rclcpp sensor_msgs cv_bridge)

target_link_libraries(insta360_node ${OpenCV_LIBS})

install(TARGETS insta360_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
