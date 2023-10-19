# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
project(custom_nitros_string LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# String Encoder node
ament_auto_add_library(string_encoder_node SHARED src/string_encoder_node.cpp)
rclcpp_components_register_nodes(string_encoder_node "custom_nitros_string::StringEncoderNode")
set(node_plugins "${node_plugins}custom_nitros_string::StringEncoderNode;$<TARGET_FILE:string_encoder_node>\n")

# String Decoder node
ament_auto_add_library(string_decoder_node SHARED src/string_decoder_node.cpp)
rclcpp_components_register_nodes(string_decoder_node "custom_nitros_string::StringDecoderNode")
set(node_plugins "${node_plugins}custom_nitros_string::StringDecoderNode;$<TARGET_FILE:string_decoder_node>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/custom_nitros_string_pol.py TIMEOUT "600")
  add_launch_test(test/custom_nitros_string_decoder_pol.py TIMEOUT "600")
  add_launch_test(test/custom_nitros_string_encoder_pol.py TIMEOUT "600")
endif()

ament_auto_package()
