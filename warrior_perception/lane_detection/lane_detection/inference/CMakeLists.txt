# CMakeLists.txt — multitask_infer
#
# Builds the TensorRT live-inference binary for the multitask perception model.
# Target platform: NVIDIA Jetson Orin (aarch64), JetPack 5.x (TensorRT 8.5/8.6, CUDA 11.4+)
# Also works on x86-64 with a local TensorRT installation.
#
# Quick start (Jetson / Linux):
#   mkdir build && cd build
#   cmake .. -DCMAKE_BUILD_TYPE=Release
#   make -j$(nproc)
#
# If TensorRT is not on the default library path, set TENSORRT_ROOT:
#   cmake .. -DTENSORRT_ROOT=/usr/local/tensorrt

cmake_minimum_required(VERSION 3.16)
project(multitask_infer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release; override with -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ─── OpenCV ──────────────────────────────────────────────────────────────────

find_package(OpenCV 4 REQUIRED
    COMPONENTS core imgproc videoio highgui)

message(STATUS "OpenCV ${OpenCV_VERSION} found at ${OpenCV_DIR}")

# ─── CUDA ────────────────────────────────────────────────────────────────────

find_package(CUDAToolkit REQUIRED)
message(STATUS "CUDA ${CUDAToolkit_VERSION} found")

# ─── TensorRT ────────────────────────────────────────────────────────────────
#
# Search order:
#   1. TENSORRT_ROOT cache variable (user-provided)
#   2. Common Jetson JetPack paths (/usr/lib/aarch64-linux-gnu, /usr/include/*)
#   3. Common x86-64 paths
#
set(TENSORRT_ROOT "" CACHE PATH
    "Root of the TensorRT installation (set if headers/libs are not in a default path)")

find_path(TRT_INCLUDE_DIR NvInfer.h
    HINTS
        ${TENSORRT_ROOT}/include
        /usr/include/aarch64-linux-gnu
        /usr/include/x86_64-linux-gnu
        /usr/local/cuda/targets/aarch64-linux/include
        /usr/include
)

find_library(TRT_NVINFER_LIB nvinfer
    HINTS
        ${TENSORRT_ROOT}/lib
        /usr/lib/aarch64-linux-gnu
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
)

find_library(TRT_NVINFER_PLUGIN_LIB nvinfer_plugin
    HINTS
        ${TENSORRT_ROOT}/lib
        /usr/lib/aarch64-linux-gnu
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
)

if(NOT TRT_INCLUDE_DIR)
    message(FATAL_ERROR
        "TensorRT headers (NvInfer.h) not found.\n"
        "Install TensorRT from developer.nvidia.com, or set:\n"
        "  cmake -DTENSORRT_ROOT=/path/to/tensorrt ..")
endif()

if(NOT TRT_NVINFER_LIB)
    message(FATAL_ERROR
        "libnvinfer not found.\n"
        "Install TensorRT or set -DTENSORRT_ROOT.")
endif()

message(STATUS "TensorRT include : ${TRT_INCLUDE_DIR}")
message(STATUS "TensorRT nvinfer : ${TRT_NVINFER_LIB}")
if(TRT_NVINFER_PLUGIN_LIB)
    message(STATUS "TensorRT plugin  : ${TRT_NVINFER_PLUGIN_LIB}")
endif()

# ─── Target ──────────────────────────────────────────────────────────────────

add_executable(multitask_infer
    src/main.cpp
    src/multitask_engine.cpp
)

target_include_directories(multitask_infer
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${TRT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
)

set(_trt_libs ${TRT_NVINFER_LIB})
if(TRT_NVINFER_PLUGIN_LIB)
    list(APPEND _trt_libs ${TRT_NVINFER_PLUGIN_LIB})
endif()

target_link_libraries(multitask_infer
    PRIVATE
        ${OpenCV_LIBS}
        ${_trt_libs}
        CUDA::cudart
)

# Compiler flags
target_compile_options(multitask_infer
    PRIVATE
        -Wall
        -Wextra
        -Wno-deprecated-declarations   # suppress TRT 8 deprecation noise
        $<$<CONFIG:Release>:-O3 -ffast-math>
        $<$<CONFIG:Debug>:-O0 -g -DDEBUG>
)

# Strip the Release binary to reduce size on Jetson flash storage
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET multitask_infer POST_BUILD
        COMMAND ${CMAKE_STRIP} $<TARGET_FILE:multitask_infer>
        COMMENT "Stripping release binary")
endif()

# ─── nlohmann/json (required by infer_images) ────────────────────────────────
#
# Install on Ubuntu / Jetson JetPack:
#   sudo apt install nlohmann-json3-dev
#
# Or let CMake fetch the header automatically via FetchContent:

find_package(nlohmann_json 3.2 QUIET)

if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found via find_package — fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(nlohmann_json
        URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
        DOWNLOAD_NO_EXTRACT TRUE
        DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/nlohmann
    )
    FetchContent_MakeAvailable(nlohmann_json)
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json
        INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/nlohmann)
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
endif()

# ─── infer_images target ──────────────────────────────────────────────────────

add_executable(infer_images
    src/infer_images.cpp
    src/multitask_engine.cpp
)

target_include_directories(infer_images
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${TRT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(infer_images
    PRIVATE
        ${OpenCV_LIBS}
        ${_trt_libs}
        CUDA::cudart
        nlohmann_json::nlohmann_json
)

target_compile_options(infer_images
    PRIVATE
        -Wall -Wextra -Wno-deprecated-declarations
        $<$<CONFIG:Release>:-O3 -ffast-math>
        $<$<CONFIG:Debug>:-O0 -g -DDEBUG>
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET infer_images POST_BUILD
        COMMAND ${CMAKE_STRIP} $<TARGET_FILE:infer_images>
        COMMENT "Stripping release binary")
endif()

# ─── Install ─────────────────────────────────────────────────────────────────

install(TARGETS multitask_infer infer_images
        RUNTIME DESTINATION bin)
