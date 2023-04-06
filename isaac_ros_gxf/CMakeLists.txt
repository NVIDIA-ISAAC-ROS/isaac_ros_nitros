# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.23.2)
project(isaac_ros_gxf)

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} STATIC src/isaac_ros_gxf.cpp)

# GXF core headers
install(
  DIRECTORY gxf/core/include/
  DESTINATION share/${PROJECT_NAME}/gxf/include/
)
# GXF ext headers
install(
  DIRECTORY gxf/ext/include/
  DESTINATION share/${PROJECT_NAME}/gxf/include/
)

# Install extensions directory
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  install(DIRECTORY gxf/core/lib/gxf_x86_64_cuda_11_8/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
  install(DIRECTORY gxf/ext/lib/gxf_x86_64_cuda_11_8/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  install(DIRECTORY gxf/core/lib/gxf_jetpack51/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
  install(DIRECTORY gxf/ext/lib/gxf_jetpack51/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
endif()

# Register cmake in install.
ament_index_register_resource(isaac_ros_gxf_cmake_path CONTENT
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake")
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/isaac_ros_gxf-extras.cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE cmake)
