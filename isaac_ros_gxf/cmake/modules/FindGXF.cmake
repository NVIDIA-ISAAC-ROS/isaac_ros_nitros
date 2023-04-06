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

# Create GXF imported cmake targets
#
# This module defines GXF_FOUND if all GXF libraries are found or
# if the required libraries (COMPONENTS property in find_package)
# are found.
#
# A new imported target is created for each component (library)
# under the GXF namespace (GXF::${component_name})
#
# Note: this leverages the find-module paradigm [1]. The config-file paradigm [2]
# is recommended instead in CMake.
# [1] https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#config-file-packages
# [2] https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#find-module-packages

# Library names
list(APPEND _GXF_EXTENSIONS
    atlas
    cuda
    isaac_messages
    isaac_ros_messages
    multimedia
    gxf_optimizer
    serialization
    std
)

list(APPEND GXF_BASE_PATHS "${isaac_ros_gxf_DIR}/../../../share/isaac_ros_gxf/gxf")

# Common headers
find_path(GXF_common_INCLUDE_DIR
    NAMES common/
    PATHS ${GXF_BASE_PATHS}
    PATH_SUFFIXES include/
    REQUIRED
)

mark_as_advanced(GXF_common_INCLUDE_DIR)
list(APPEND GXF_INCLUDE_DIR_VARS GXF_common_INCLUDE_DIR)

# Libraries and their headers
list(APPEND _GXF_LIBRARIES ${_GXF_EXTENSIONS} core)

foreach(component IN LISTS _GXF_LIBRARIES)
    # headers
    find_path(GXF_${component}_INCLUDE_DIR
        NAMES "${component}/"
        PATHS ${GXF_BASE_PATHS}
        PATH_SUFFIXES
            include
            include/gxf
        NO_DEFAULT_PATH
        REQUIRED
    )
    mark_as_advanced(GXF_${component}_INCLUDE_DIR)
    list(APPEND GXF_INCLUDE_DIR_VARS GXF_${component}_INCLUDE_DIR)

    # library
    find_library(GXF_${component}_LIBRARY
        NAMES "gxf_${component}" "${component}"
        PATHS ${GXF_BASE_PATHS}
        PATH_SUFFIXES
            lib/${component}
            lib/${GXF_PLATFORM_SUFFIX}
            lib/${GXF_PLATFORM_SUFFIX}/${component}
            lib/${GXF_PLATFORM_SUFFIX}/isaac
            lib/${GXF_PLATFORM_SUFFIX}/isaac/${component}
        REQUIRED
    )
    mark_as_advanced(GXF_${component}_LIBRARY)
    list(APPEND GXF_LIBRARY_VARS GXF_${component}_LIBRARY)

    # create imported target
    if(GXF_${component}_LIBRARY AND GXF_${component}_INCLUDE_DIR)
        set(gxf_component_location "${GXF_${component}_LIBRARY}")

        if(NOT TARGET GXF::${component})
            # Assume SHARED, though technically UNKNOWN since we don't enforce .so
            add_library(GXF::${component} SHARED IMPORTED)
        endif()

        list(APPEND GXF_${component}_INCLUDE_DIRS
            ${GXF_${component}_INCLUDE_DIR}
            ${GXF_${component}_INCLUDE_DIR}/${component}
            ${GXF_common_INCLUDE_DIR}
        )

        message(STATUS "Found ${component} lib=${gxf_component_location}, include=${GXF_${component}_INCLUDE_DIR}")
        # Points to the copied location of GXF
        set_target_properties(GXF::${component} PROPERTIES
            IMPORTED_LOCATION "${gxf_component_location}"
            IMPORTED_NO_SONAME ON
            INTERFACE_INCLUDE_DIRECTORIES "${GXF_${component}_INCLUDE_DIRS}"
        )

        set(GXF_${component}_FOUND TRUE)
    else()
        message(FATAL_ERROR "Could not find GXF::${component}: lib={GXF_${component}_LIBRARY} / include=${GXF_${component}_INCLUDE_DIR}")
        set(GXF_${component}_FOUND FALSE)
    endif()
endforeach()

unset(_GXF_EXTENSIONS)
unset(_GXF_LIBRARIES)

# Find version
if(GXF_core_INCLUDE_DIR)
    # Note: "kGxfCoreVersion \"(.*)\"$" does not work with a simple string
    # REGEX (doesn't stop and EOL, neither $ nor \n), so we first extract
    # the line with file(STRINGS), then the version with string(REGEX)
    file(STRINGS "${GXF_core_INCLUDE_DIR}/core/gxf.h" _GXF_VERSION_LINE
        REGEX "kGxfCoreVersion"
    )
    string(REGEX MATCH "kGxfCoreVersion \"(.*)\"" _ ${_GXF_VERSION_LINE})
    set(GXF_VERSION ${CMAKE_MATCH_1})
    unset(_GXF_VERSION_LINE)
endif()

# Generate GXF_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GXF
    FOUND_VAR GXF_FOUND
    VERSION_VAR GXF_VERSION
    HANDLE_COMPONENTS # Looks for GXF_${component}_FOUND
)
