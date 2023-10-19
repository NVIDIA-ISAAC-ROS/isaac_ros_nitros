// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/type_utility.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// The base struct for all Nitros-based data types/formats
struct NitrosTypeBase
{
  NitrosTypeBase() = default;

  // Constructor
  NitrosTypeBase(
    const int64_t handle,
    const std::string data_format_name,
    const std::string compatible_data_format_name,
    const std::string frame_id);

  // Copy constructor
  NitrosTypeBase(const NitrosTypeBase & other);

  // Destructor
  ~NitrosTypeBase();

  int64_t handle;
  std::string data_format_name;
  std::string compatible_data_format_name;
  std::string frame_id;
};

/* *INDENT-OFF* */
// Factory macros that help define NITROS data type
// Mark the beginning of a factory for TYPE_TYPENAME
#define NITROS_TYPE_FACTORY_BEGIN(TYPE_TYPENAME) \
struct TYPE_TYPENAME : NitrosTypeBase \
{ \
  using NitrosTypeBase::NitrosTypeBase;

// Mark the end of the type factory
#define NITROS_TYPE_FACTORY_END() \
};

// Mark the beginning of a format factory for the current data type
#define NITROS_FORMAT_FACTORY_BEGIN() \
static std::map<std::string, NitrosFormatCallbacks> GetFormatCallbacks() \
{ \
  std::map<std::string, NitrosFormatCallbacks> format_callback_map;

// Add a supported format under the current data type
#define NITROS_FORMAT_ADD(FORMAT_TYPENAME) \
  format_callback_map.emplace( \
    FORMAT_TYPENAME::supported_type_name, \
    NitrosFormatAgent<FORMAT_TYPENAME>::GetFormatCallbacks());

// Mark the end of the format factory
#define NITROS_FORMAT_FACTORY_END() \
  return format_callback_map; \
}

// Mark the beginning of an extension factory for the current data type
#define NITROS_TYPE_EXTENSION_FACTORY_BEGIN() \
static std::vector<std::pair<std::string, std::string>> GetExtensions() \
{ \
  std::vector<std::pair<std::string, std::string>> extensions;

// Add a required extension for the current data type
#define NITROS_TYPE_EXTENSION_ADD(PACKAGE_NAME, SO_FILE_RELATIVE_PATH) \
  extensions.push_back({PACKAGE_NAME, SO_FILE_RELATIVE_PATH});

// Mark the end of the extension factory
#define NITROS_TYPE_EXTENSION_FACTORY_END() \
  return extensions; \
}
/* *INDENT-ON* */

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_
