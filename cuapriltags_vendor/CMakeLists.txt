cmake_minimum_required(VERSION 3.22.1)
project(cuapriltags_vendor)

find_package(ament_cmake REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(CUAPRILTAGS_LIBRARY_DIR "lib_aarch64_jetpack61")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(CUAPRILTAGS_LIBRARY_DIR "lib_x86_64_cuda_12_6")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

install(
  FILES VERSION
  DESTINATION "share/${PROJECT_NAME}/cuapriltags"
)
install(
  FILES cuapriltags/cuAprilTags.h
  DESTINATION "share/${PROJECT_NAME}/cuapriltags/cuapriltags"
)
install(
  FILES "${CUAPRILTAGS_LIBRARY_DIR}/libcuapriltags.a"
  DESTINATION "share/${PROJECT_NAME}/cuapriltags/lib"
)

ament_index_register_resource(
  cuapriltags
  CONTENT "share/${PROJECT_NAME}/cuapriltags"
)

ament_package()
