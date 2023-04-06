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

# Lock YAML-CPP to 0.6.3 for compatibility
set(ISAAC_ROS_GXF_YAML_CPP_VERSION 0.6.3)

# Ensure that the following flags are picked up
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(YAML_CPP_BUILD_TESTS OFF)
set(YAML_CPP_BUILD_TOOLS OFF)

# Lock version of yaml-cpp for compatibility
include(FetchContent)
fetchcontent_declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-${ISAAC_ROS_GXF_YAML_CPP_VERSION}
)
fetchcontent_makeavailable(yaml-cpp)

# Ignore tests from yaml-cpp
file(TOUCH ${yaml-cpp_SOURCE_DIR}/AMENT_IGNORE)

# Suppress all warnings from yaml-cpp
target_compile_options(yaml-cpp PUBLIC -w)
