# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

include(CMakeFindDependencyMacro)

find_dependency(CUDAToolkit)
find_dependency(vpi)

# cuapriltags is consumed as an ament resource by downstream packages and is
# linked as a static archive. Export its system link dependencies through the
# isaac_ros_nitros package so ament_target_dependencies() consumers receive
# the CUDA and VPI link flags without redefining their local cuapriltags target.
list(APPEND isaac_ros_nitros_LIBRARIES CUDA::cudart vpi)
