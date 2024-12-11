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
project(isaac_ros_nitros_bridge_ros1)

if($ENV{ROS_VERSION} EQUAL 2)
  message(STATUS "Skip package ${PROJECT_NAME} for ROS 2")
  return()
endif()

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             roscpp
             nodelet)
find_package(isaac_ros_nitros_bridge_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(CMAKE_CUDA_ARCHITECTURES "89;87;86;80;75;72;70;61;60")

#find_package(cv_bridge REQUIRED)
#find_package(OpenCV 4 REQUIRED COMPONENTS opencv_core opencv_imgproc opencv_imgcodecs)

include_directories( 
    include 
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${isaac_ros_nitros_bridge_msgs_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs isaac_ros_nitros_bridge_msgs)

#############
## Library ##
#############

add_library(isaac_ros_nitros_bridge_ros1
    src/image_converter_node.cpp)

target_link_libraries(isaac_ros_nitros_bridge_ros1 
    ${catkin_LIBRARIES} 
    CUDA::cudart
    CUDA::cuda_driver
)

#############
## Install ##
#############

install(TARGETS isaac_ros_nitros_bridge_ros1
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
  launch/nitros_bridge_convert_forward.launch
  launch/nitros_bridge_forward.launch
  launch/nitros_bridge_image_converter.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)