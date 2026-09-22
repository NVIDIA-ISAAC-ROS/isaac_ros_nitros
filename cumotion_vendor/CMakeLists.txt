cmake_minimum_required(VERSION 3.22.1)
project(cumotion_vendor)

find_package(ament_cmake REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(CUMOTION_PLATFORM_DIR "aarch64_jetpack70")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(CUMOTION_PLATFORM_DIR "x86_64_cuda_13_0")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

install(
  DIRECTORY "${CUMOTION_PLATFORM_DIR}/"
  DESTINATION "share/${PROJECT_NAME}/cumotion"
  USE_SOURCE_PERMISSIONS
)

ament_index_register_resource(
  cumotion
  CONTENT "share/${PROJECT_NAME}/cumotion"
)

ament_package()
