# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
