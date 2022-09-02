# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.8)
project(isaac_ros_nitros LANGUAGES C CXX)


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


# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# GXF
include_directories(gxf/include)
include_directories(gxf/include/third_party)
add_library(gxf SHARED IMPORTED)
add_library(gxf_std SHARED IMPORTED)
add_library(gxf_multimedia SHARED IMPORTED)  # for NitrosImage
add_library(optimizer SHARED IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET gxf PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_x86_64/core/libgxf_core.so)
  set_property(TARGET gxf_std PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_x86_64/std/libgxf_std.so)
  set_property(
    TARGET gxf_multimedia PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_x86_64/multimedia/libgxf_multimedia.so)
  set_property(TARGET optimizer PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_x86_64/libgxf_optimizer.so)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET gxf PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_jetpack50/core/libgxf_core.so)
  set_property(TARGET gxf_std PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_jetpack50/std/libgxf_std.so)
  set_property(
    TARGET gxf_multimedia PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_jetpack50/multimedia/libgxf_multimedia.so)
  set_property(TARGET optimizer PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/gxf/lib/gxf_jetpack50/libgxf_optimizer.so)
endif()


# for #include <cuda_runtime.h>
find_package(CUDA REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

option(USE_NVTX "Enable NVTX markers for improved profiling (if available)" ON)
if(USE_NVTX)
    add_definitions(-DUSE_NVTX)
    link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
    link_libraries("nvToolsExt")
endif()


# NitrosNode
ament_auto_add_library(isaac_ros_nitros SHARED
  src/nitros_node.cpp
  src/nitros_context.cpp
  src/types/type_adapter_nitros_context.cpp
  src/types/nitros_type_base.cpp
  src/types/nitros_int64.cpp
  src/nitros_publisher.cpp
  src/nitros_subscriber.cpp
  src/nitros_publisher_subscriber_group.cpp)
target_compile_definitions(isaac_ros_nitros
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(isaac_ros_nitros gxf gxf_multimedia optimizer Eigen3::Eigen)
# For the NitrosNode test node
rclcpp_components_register_node(isaac_ros_nitros
  PLUGIN "nvidia::isaac_ros::nitros::NitrosNode"
  EXECUTABLE isaac_ros_nitros_exe)

# Nitros headers
install(
  DIRECTORY include/
  DESTINATION include/
)
# GXF headers
install(
  DIRECTORY gxf/include/
  DESTINATION include/
)
# GXF third party headers
install(
  DIRECTORY gxf/include/third_party/
  DESTINATION include/
)
# Isaac headers
install(
  DIRECTORY gxf/include/extensions/isaac/
  DESTINATION include/
)
ament_export_include_directories("include")


# Install config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Install extensions directory
# Also copying libgxf_core.so to the lib directory for exposing it to other packages
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  install(DIRECTORY gxf/lib/gxf_x86_64/ DESTINATION share/${PROJECT_NAME}/gxf)
  install(FILES gxf/lib/gxf_x86_64/multimedia/libgxf_multimedia.so DESTINATION lib/)
  install(FILES gxf/lib/gxf_x86_64/cuda/libgxf_cuda.so DESTINATION lib/)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  install(DIRECTORY gxf/lib/gxf_jetpack50/ DESTINATION share/${PROJECT_NAME}/gxf)
  install(FILES gxf/lib/gxf_jetpack50/multimedia/libgxf_multimedia.so DESTINATION lib/)
  install(FILES gxf/lib/gxf_jetpack50/cuda/libgxf_cuda.so DESTINATION lib/)
endif()
set_target_properties(isaac_ros_nitros PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(isaac_ros_nitros PROPERTIES INSTALL_RPATH "$ORIGIN/../share/${PROJECT_NAME}/gxf/core")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_test_pol.py TIMEOUT "15")
endif()

ament_auto_package()
