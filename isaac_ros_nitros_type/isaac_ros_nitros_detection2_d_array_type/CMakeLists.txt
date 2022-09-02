# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.8)
project(isaac_ros_nitros_detection2_d_array_type LANGUAGES C CXX)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Detectnet
include_directories(gxf/include)
include_directories(gxf/include/extensions)
add_library(detectnet SHARED IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET detectnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_x86_64/libgxf_detectnet.so)
  install(DIRECTORY gxf/lib/gxf_x86_64/ DESTINATION share/${PROJECT_NAME}/gxf)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET detectnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_jetpack50/libgxf_detectnet.so)
  install(DIRECTORY gxf/lib/gxf_jetpack50/ DESTINATION share/${PROJECT_NAME}/gxf)
endif()
# Detect headers
install(
  DIRECTORY gxf/include/extensions
  DESTINATION include/
)

# for #include <cuda_runtime.h>
find_package(CUDA REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

option(USE_NVTX "Enable NVTX markers for improved profiling (if available)" ON)
if(USE_NVTX)
    add_definitions(-DUSE_NVTX)
    link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
    link_libraries("nvToolsExt")
endif()


# NitrosDetection2DArray
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nitros_detection2_d_array.cpp)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(${PROJECT_NAME} detectnet)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_include_directories("include")


if(BUILD_TESTING)
  # NitrosDetection2DArrayForwardNode
  ament_auto_add_library(
    isaac_ros_nitros_detection2_d_array_forward_node SHARED test/src/nitros_detection2_d_array_forward_node.cpp)
  target_compile_definitions(isaac_ros_nitros_detection2_d_array_forward_node
    PRIVATE "COMPOSITION_BUILDING_DLL"
  )
  target_link_libraries(isaac_ros_nitros_detection2_d_array_forward_node ${PROJECT_NAME})
  rclcpp_components_register_nodes(
    isaac_ros_nitros_detection2_d_array_forward_node "nvidia::isaac_ros::nitros::NitrosDetection2DArrayForwardNode")
  set(node_plugins
    "${node_plugins}nvidia::isaac_ros::nitros::NitrosDetection2DArrayForwardNode;$<TARGET_FILE:isaac_ros_nitros_detection2_d_array_forward_node>\n")
  # Install test/config directory
  install(
    DIRECTORY test/config
    DESTINATION share/${PROJECT_NAME}/test
  )

  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_detection2_d_array_type_test_pol.py TIMEOUT "15")
endif()

ament_auto_package()
