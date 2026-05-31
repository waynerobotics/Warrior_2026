cmake_minimum_required(VERSION 3.16)
project(neural_engine)

# -----------------------------------------------------------------------------
# Global build settings
# -----------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Shared compile options.
set(NE_COMMON_WARNINGS
    -Wall
    -Wextra
)

set(NE_RELEASE_FAST_FLAGS
    $<$<CONFIG:Release>:-O3>
)

set(NE_RELEASE_FAST_MATH_FLAGS
    $<$<CONFIG:Release>:-O3 -ffast-math>
)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(vision_msgs    REQUIRED)
find_package(cv_bridge      REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)
find_package(CUDAToolkit REQUIRED)

# -----------------------------------------------------------------------------
# TensorRT discovery
# -----------------------------------------------------------------------------

set(TENSORRT_ROOT "" CACHE PATH
    "Root of the TensorRT installation (set if not on the default path)")

set(_trt_include_hints
    ${TENSORRT_ROOT}/include
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/include
)

set(_trt_library_hints
    ${TENSORRT_ROOT}/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

find_path(TRT_INCLUDE_DIR NvInfer.h
    HINTS ${_trt_include_hints}
)

find_library(TRT_NVINFER_LIB nvinfer
    HINTS ${_trt_library_hints}
)

find_library(TRT_NVINFER_PLUGIN_LIB nvinfer_plugin
    HINTS ${_trt_library_hints}
)

if(NOT TRT_INCLUDE_DIR OR NOT TRT_NVINFER_LIB)
    message(FATAL_ERROR
        "TensorRT not found. Install from developer.nvidia.com or set:\n"
        "  colcon build --cmake-args -DTENSORRT_ROOT=/path/to/tensorrt")
endif()

# Build TensorRT link list once.
set(_trt_libs ${TRT_NVINFER_LIB})
if(TRT_NVINFER_PLUGIN_LIB)
    list(APPEND _trt_libs ${TRT_NVINFER_PLUGIN_LIB})
endif()

# -----------------------------------------------------------------------------
# multitask_node
# -----------------------------------------------------------------------------

add_executable(multitask_node
    src/multitask_node.cpp
    src/inference/multitask_engine.cpp
)

target_include_directories(multitask_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TRT_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

ament_target_dependencies(multitask_node
    rclcpp
    sensor_msgs
    vision_msgs
    cv_bridge
)

target_link_libraries(multitask_node
    ${OpenCV_LIBS}
    ${_trt_libs}
    CUDA::cudart
)

target_compile_options(multitask_node PRIVATE
    ${NE_COMMON_WARNINGS}
    -Wno-deprecated-declarations
    ${NE_RELEASE_FAST_MATH_FLAGS}
)

# -----------------------------------------------------------------------------
# seg_viz_node
# -----------------------------------------------------------------------------

add_executable(seg_viz_node src/seg_viz_node.cpp)

ament_target_dependencies(seg_viz_node
    rclcpp
    sensor_msgs
    cv_bridge
)

target_link_libraries(seg_viz_node ${OpenCV_LIBS})

target_compile_options(seg_viz_node PRIVATE
    ${NE_COMMON_WARNINGS}
    ${NE_RELEASE_FAST_FLAGS}
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------

install(TARGETS multitask_node seg_viz_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

set(_engine_files)
foreach(_engine IN ITEMS
    models/resnet_fp32.engine
    models/resnet18.engine
)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_engine}")
        list(APPEND _engine_files ${_engine})
    endif()
endforeach()

if(_engine_files)
    install(FILES ${_engine_files}
        DESTINATION share/${PROJECT_NAME}/models)
endif()

ament_package()
