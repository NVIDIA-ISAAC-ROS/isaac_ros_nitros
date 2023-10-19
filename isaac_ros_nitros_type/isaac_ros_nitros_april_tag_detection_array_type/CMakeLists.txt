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
project(isaac_ros_nitros_april_tag_detection_array_type LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# NVTX
option(USE_NVTX "Enable NVTX markers for improved profiling (if available)" ON)
if(USE_NVTX)
    add_definitions(-DUSE_NVTX)
    link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
    link_libraries("nvToolsExt")
endif()

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    COMPONENTS
    core
    isaac_messages
    multimedia
)
find_package(yaml-cpp)

# NitrosAprilTagDetectionArray
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nitros_april_tag_detection_array.cpp
  src/fiducial_message.cpp
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  GXF::core
  GXF::isaac_messages
  GXF::multimedia
  yaml-cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Make library available for sharing in Nitros
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION share/${PROJECT_NAME}/gxf/lib/fiducial_message)

if(BUILD_TESTING)
  # Install test/config directory
  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  # NitrosAprilTagDetectionArrayForwardNode
  ament_auto_add_library(isaac_ros_nitros_april_tag_detection_array_forward_node SHARED
    test/src/nitros_april_tag_detection_array_forward_node.cpp
  )
  target_link_libraries(isaac_ros_nitros_april_tag_detection_array_forward_node ${PROJECT_NAME} GXF::std)
  set_target_properties(isaac_ros_nitros_april_tag_detection_array_forward_node PROPERTIES
      BUILD_RPATH_USE_ORIGIN TRUE)
  set_target_properties(isaac_ros_nitros_april_tag_detection_array_forward_node PROPERTIES
      INSTALL_RPATH_USE_LINK_PATH TRUE)
  rclcpp_components_register_nodes(isaac_ros_nitros_april_tag_detection_array_forward_node
      "nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArrayForwardNode")
  set(node_plugins "${node_plugins}nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArrayForwardNode;\
      $<TARGET_FILE:isaac_ros_nitros_april_tag_detection_array_forward_node>\n")

  # Test registration extension
  ament_auto_add_library(isaac_ros_nitros_april_tag_detection_array_test_ext SHARED
      test/src/fiducial_message_test_ext.cpp
  )
  target_link_libraries(isaac_ros_nitros_april_tag_detection_array_test_ext GXF::std)
  install(TARGETS isaac_ros_nitros_april_tag_detection_array_test_ext
      LIBRARY DESTINATION share/${PROJECT_NAME}/gxf/lib/fiducial_message/test)

  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_april_tag_detection_array_type_test_pol.py TIMEOUT "15")
endif()

ament_auto_package()
