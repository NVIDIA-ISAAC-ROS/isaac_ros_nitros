# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
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

# Install extensions directory
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  install(DIRECTORY gxf/core/lib/gxf_x86_64_cuda_12_6/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
  install(FILES gxf/core/lib/gxf_x86_64_cuda_12_6/core/libgxf_core.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_x86_64_cuda_12_6/logger/libgxf_logger.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_x86_64_cuda_12_6/multimedia/libgxf_multimedia.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_x86_64_cuda_12_6/cuda/libgxf_cuda.so DESTINATION lib)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  install(DIRECTORY gxf/core/lib/gxf_jetpack61/ DESTINATION share/${PROJECT_NAME}/gxf/lib)
  install(FILES gxf/core/lib/gxf_jetpack61/core/libgxf_core.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_jetpack61/logger/libgxf_logger.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_jetpack61/multimedia/libgxf_multimedia.so DESTINATION lib)
  install(FILES gxf/core/lib/gxf_jetpack61/cuda/libgxf_cuda.so DESTINATION lib)
endif()

# Register cmake in install.
ament_index_register_resource(isaac_ros_gxf_cmake_path CONTENT
    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake")
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/isaac_ros_gxf-extras.cmake)

# Core
add_library(Core INTERFACE)
set_target_properties(Core PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)
set_property(TARGET Core PROPERTY
  INTERFACE_LINK_LIBRARIES
    magic_enum::magic_enum
    "$<INSTALL_PREFIX>/lib/libgxf_core.so"
)

# Logger
add_library(Logger INTERFACE)
set_target_properties(Logger PROPERTIES
  INTERFACE_LINK_LIBRARIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/logger/libgxf_logger.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)

# Std
add_library(Std INTERFACE)
set_target_properties(Std PROPERTIES
  INTERFACE_LINK_LIBRARIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/std/libgxf_std.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)

# Multimedia
add_library(Multimedia INTERFACE)
set_target_properties(Multimedia PROPERTIES
  INTERFACE_LINK_LIBRARIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/multimedia/libgxf_multimedia.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)

# Serialization
add_library(Serialization INTERFACE)
set_target_properties(Serialization PROPERTIES
  INTERFACE_LINK_LIBRARIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/serialization/libgxf_serialization.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)

# Cuda
add_library(Cuda INTERFACE)
set_target_properties(Cuda PROPERTIES
  INTERFACE_LINK_LIBRARIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/cuda/libgxf_cuda.so"
  INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include"
)

install(TARGETS Core Logger Std Multimedia Serialization Cuda
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE cmake)
