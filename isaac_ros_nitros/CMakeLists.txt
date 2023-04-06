# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.23.2)
project(isaac_ros_nitros LANGUAGES C CXX)

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

# Elbrus (register path and install)
ament_index_register_resource(elbrus CONTENT share/${PROJECT_NAME}/elbrus)
install(DIRECTORY lib/elbrus
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# nvapriltags (register path and install)
ament_index_register_resource(nvapriltags CONTENT share/${PROJECT_NAME}/nvapriltags)
install(DIRECTORY lib/nvapriltags
  DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    COMPONENTS
    core
    multimedia
    gxf_optimizer
)
find_package(vpi REQUIRED)
include(YamlCpp)

# NitrosNode
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/nitros_node.cpp
  src/nitros_context.cpp
  src/types/type_adapter_nitros_context.cpp
  src/types/nitros_type_base.cpp
  src/types/nitros_empty.cpp
  src/nitros_publisher.cpp
  src/nitros_subscriber.cpp
  src/nitros_publisher_subscriber_group.cpp
  src/utils/vpi_utilities.cpp
)
# For the NitrosNode test node
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "nvidia::isaac_ros::nitros::NitrosNode"
  EXECUTABLE ${PROJECT_NAME}_exe)

target_link_libraries(${PROJECT_NAME}
  GXF::core
  GXF::gxf_optimizer
  GXF::multimedia
  Eigen3::Eigen
  vpi
  yaml-cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Re-export gxf_optimizer
get_target_property(gxf_optimizer_includes GXF::gxf_optimizer INTERFACE_INCLUDE_DIRECTORIES)
ament_export_include_directories(${gxf_optimizer_includes})

# Re-export isaac_messages headers embedded directory
find_package(isaac_ros_gxf)
ament_export_include_directories("${isaac_ros_gxf_DIR}/../gxf/include/gxf/isaac_messages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom NVIDIA Isaac ROS Software License
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_nitros_test_pol.py TIMEOUT "15")
endif()

ament_auto_package(INSTALL_TO_SHARE config)
