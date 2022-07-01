/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_

#include <string>

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

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_BASE_HPP_
