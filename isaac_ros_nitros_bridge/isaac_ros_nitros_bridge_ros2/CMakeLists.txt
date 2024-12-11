# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(isaac_ros_nitros_bridge_ros2 LANGUAGES C CXX)

if($ENV{ROS_VERSION} EQUAL 1)
  message(STATUS "Skip package ${PROJECT_NAME} for ROS 1")
  return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(CUDAToolkit REQUIRED)
ament_auto_find_build_dependencies()

# Image converter node
ament_auto_add_library(image_converter_node SHARED src/image_converter_node.cpp)
target_link_libraries(image_converter_node CUDA::cuda_driver)
set_target_properties(image_converter_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

rclcpp_components_register_nodes(image_converter_node "nvidia::isaac_ros::nitros_bridge::ImageConverterNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::nitros_bridge::ImageConverterNode;$<TARGET_FILE:image_converter_node>\n")


# TensorList converter node
ament_auto_add_library(tensor_list_converter_node SHARED src/tensor_list_converter_node.cpp)
target_link_libraries(tensor_list_converter_node CUDA::cuda_driver)
set_target_properties(tensor_list_converter_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

rclcpp_components_register_nodes(tensor_list_converter_node "nvidia::isaac_ros::nitros_bridge::TensorListConverterNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::nitros_bridge::TensorListConverterNode;$<TARGET_FILE:tensor_list_converter_node>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_bridge_image_pol.py TIMEOUT "15")
  add_launch_test(test/isaac_ros_nitros_bridge_tensor_list_pol.py TIMEOUT "15")
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch)
