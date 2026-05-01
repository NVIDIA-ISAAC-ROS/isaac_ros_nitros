# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES.
#                         All rights reserved.
# SPDX-License-Identifier: Apache-2.0
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


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was CMakeConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

include(CMakeFindDependencyMacro)
find_dependency(Eigen3 3.3)
find_dependency(CUDAToolkit)

set(CUDA_PATH ${CUDAToolkit_LIBRARY_ROOT})
if (${CUDAToolkit_VERSION} VERSION_GREATER_EQUAL 13.0)
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CCCL_DIR "${CUDA_PATH}/targets/sbsa-linux/lib/cmake/cccl")  # for Jetpack 7.0
  else()
    set(CCCL_DIR "${CUDA_PATH}/lib64/cmake/cccl")
  endif()
  find_dependency(CCCL REQUIRED)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cumotionConfigVersion.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cumotionTargets.cmake)
