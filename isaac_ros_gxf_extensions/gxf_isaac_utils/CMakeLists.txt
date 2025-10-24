# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.5)
project(gxf_isaac_utils LANGUAGES C CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-gpu-targets)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Dependencies
find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp)

# Import the original Isaac's utils extension as library
add_library(libgxf_utils SHARED IMPORTED)
# Determine the architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  if(CMAKE_DEVICE STREQUAL "sbsa")
    set(GXF_EXT_LIB_PATH "lib/gxf_aarch64_cuda_13_0")
  elseif(CMAKE_DEVICE STREQUAL "arm64")
    set(GXF_EXT_LIB_PATH "lib/gxf_jetpack70")
  else()
    message(FATAL_ERROR "Device is not supported.")
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(GXF_EXT_LIB_PATH "lib/gxf_x86_64_cuda_13_0")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set_property(TARGET libgxf_utils PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${GXF_EXT_LIB_PATH}/libgxf_utils.so)

# gxf_isaac_uitls extension
ament_auto_add_library(gxf_isaac_utils SHARED
  gxf/extensions/utils/utils.cpp
  gxf/extensions/utils/disparity_to_depth.cpp
  gxf/extensions/utils/disparity_to_depth.cu.cpp
  gxf/extensions/utils/image_loader.cpp
  gxf/extensions/utils/udp_receiver.cpp
)
# Mark as CUDA files with non-standard extensions
set_source_files_properties(
  gxf/extensions/utils/disparity_to_depth.cu.cpp
  gxf/extensions/utils/disparity_to_depth.cu.hpp
  PROPERTIES LANGUAGE CUDA
)
target_include_directories(gxf_isaac_utils PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/gxf")
target_link_libraries(gxf_isaac_utils
    CUDA::cudart
    Eigen3::Eigen
    yaml-cpp
    libgxf_utils
)
set_target_properties(gxf_isaac_utils PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "$ORIGIN:$ORIGIN/../share/${PROJECT_NAME}/gxf/lib") # find libgxf_utils.so in share/

# Disable -Wpedantic flag
get_target_property(target_options gxf_isaac_utils COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-Wpedantic")
set_property(TARGET gxf_isaac_utils PROPERTY COMPILE_OPTIONS ${target_options})

# Install libgxf_isaac_utils.so in share/
install(TARGETS gxf_isaac_utils DESTINATION share/${PROJECT_NAME}/gxf/lib)
# Install libgxf_utils.so dependency in share/
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${GXF_EXT_LIB_PATH}/libgxf_utils.so
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gxf/lib)


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE)
