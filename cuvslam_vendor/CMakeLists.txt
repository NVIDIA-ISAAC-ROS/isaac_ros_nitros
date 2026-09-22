cmake_minimum_required(VERSION 3.22.1)
project(cuvslam_vendor)

find_package(ament_cmake REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(CUVSLAM_LIBRARY_DIR "lib_aarch64_jetpack70")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(CUVSLAM_LIBRARY_DIR "lib_x86_64_cuda_13_0")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

install(
  DIRECTORY include/
  DESTINATION "share/${PROJECT_NAME}/cuvslam/include"
)
install(
  FILES "${CUVSLAM_LIBRARY_DIR}/libcuvslam.so"
  DESTINATION "share/${PROJECT_NAME}/cuvslam/lib"
)
install(
  PROGRAMS "${CUVSLAM_LIBRARY_DIR}/cuvslam_api_launcher"
  DESTINATION "share/${PROJECT_NAME}/cuvslam/lib"
)

ament_index_register_resource(
  cuvslam
  CONTENT "share/${PROJECT_NAME}/cuvslam"
)

ament_package()
