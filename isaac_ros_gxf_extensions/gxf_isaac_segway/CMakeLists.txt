# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(gxf_isaac_segway LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Determine the architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(GXF_EXT_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/gxf_jetpack61")
else()
  message(WARNING
          "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}. "
          "${PROJECT_NAME} is only supported on aarch64 platforms.")
endif()

# Add an interface target to export dependencies
add_library(${PROJECT_NAME} INTERFACE)

# Install shared library
if(EXISTS "${GXF_EXT_LIB_PATH}/lib${PROJECT_NAME}.so")
  install(FILES "${GXF_EXT_LIB_PATH}/lib${PROJECT_NAME}.so"
          DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gxf/lib")
  set_property(TARGET ${PROJECT_NAME} PROPERTY
    INTERFACE_LINK_LIBRARIES
      "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/lib/lib${PROJECT_NAME}.so"
  )
endif()

# Install headers
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gxf")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/gxf/"
          DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gxf/include")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
      "$<INSTALL_PREFIX>/share/${PROJECT_NAME}/gxf/include")
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE)