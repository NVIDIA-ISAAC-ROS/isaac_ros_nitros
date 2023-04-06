# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Common flags and cmake commands for all Isaac ROS packages based on NITROS

message(STATUS "Loading isaac_ros_gxf extras")

# Append local cmake module path for CMAKE_MODULE_PATH
ament_index_get_resource(ISAAC_ROS_GXF_CMAKE_PATH isaac_ros_gxf_cmake_path isaac_ros_gxf)
list(APPEND CMAKE_MODULE_PATH "${ISAAC_ROS_GXF_CMAKE_PATH}/modules")

# Versions
set(ISAAC_ROS_GXF_VERSION 2.5.0)