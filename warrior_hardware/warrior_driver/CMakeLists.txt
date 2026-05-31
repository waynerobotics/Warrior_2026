cmake_minimum_required(VERSION 3.16)
project(warrior_driver CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS
  rclcpp
  std_msgs
  std_srvs
  builtin_interfaces
  diagnostic_msgs
  warrior_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

find_package(Threads REQUIRED)

add_executable(warrior_driver_node
  src/main.cpp
  src/swerve/swerve_driver_node.cpp
  src/arduino/arduino_serial_device.cpp
  src/sparkmax/sparkmax_session.cpp
  src/swerve/device_registry.cpp
)
target_link_libraries(warrior_driver_node Threads::Threads)
target_include_directories(warrior_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(warrior_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(warrior_driver_node ${THIS_PACKAGE_DEPS})

add_executable(nudge_sparks_cli
  src/sparkmax/nudge_sparks_cli.cpp
  src/sparkmax/sparkmax_session.cpp
)
target_link_libraries(nudge_sparks_cli Threads::Threads)
target_include_directories(nudge_sparks_cli PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(nudge_sparks_cli PUBLIC cxx_std_17)

install(TARGETS warrior_driver_node nudge_sparks_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
