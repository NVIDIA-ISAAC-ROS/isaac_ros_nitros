# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.23.2)
project(isaac_ros_nitros_detection2_d_array_type LANGUAGES C CXX)

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
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    COMPONENTS
    core
    std
)
include(YamlCpp)

# NitrosDetection2DArray
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nitros_detection2_d_array.cpp
  src/detection2_d_array_message.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include/detectnet)
target_link_libraries(${PROJECT_NAME}
  GXF::core
  yaml-cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Make library available for sharing in Nitros
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION share/${PROJECT_NAME}/gxf/lib/detection2_d)
ament_export_include_directories(include/detectnet)

if(BUILD_TESTING)
  # Install test/config directory
  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  # NitrosDetection2DArrayForwardNode
  ament_auto_add_library(isaac_ros_nitros_detection2_d_array_forward_node SHARED
      test/src/nitros_detection2_d_array_forward_node.cpp
  )
  target_link_libraries(isaac_ros_nitros_detection2_d_array_forward_node ${PROJECT_NAME} GXF::std)
  set_target_properties(isaac_ros_nitros_detection2_d_array_forward_node PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
  set_target_properties(isaac_ros_nitros_detection2_d_array_forward_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  rclcpp_components_register_nodes(isaac_ros_nitros_detection2_d_array_forward_node
      "nvidia::isaac_ros::nitros::NitrosDetection2DArrayForwardNode")
  set(node_plugins "${node_plugins}nvidia::isaac_ros::nitros::NitrosDetection2DArrayForwardNode;\
      $<TARGET_FILE:isaac_ros_nitros_detection2_d_array_forward_node>\n")

  # Test registration extension
  ament_auto_add_library(isaac_ros_nitros_detection2_d_array_test_ext SHARED
      test/src/detection2_d_test_ext.cpp
  )
  target_link_libraries(isaac_ros_nitros_detection2_d_array_test_ext GXF::std)
  install(TARGETS isaac_ros_nitros_detection2_d_array_test_ext LIBRARY DESTINATION share/${PROJECT_NAME}/gxf/lib/detection2_d/test)

  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)

  # Ignore lint warnings from GXF source
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    include/detectnet/detection2_d.hpp
    include/detectnet/detection2_d_array_message.hpp
  )

  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_detection2_d_array_type_test_pol.py TIMEOUT "15")
endif()

ament_auto_package()
