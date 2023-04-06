/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__TYPE_ADAPTER_NITROS_CONTEXT_HPP_
#define ISAAC_ROS_NITROS__TYPES__TYPE_ADAPTER_NITROS_CONTEXT_HPP_

#include <memory>
#include <mutex>

#include "isaac_ros_nitros/nitros_context.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Global NitrosContext for data type adapter
extern std::unique_ptr<NitrosContext> g_type_adapter_nitros_context;

// Mutex for the type adapter's global NitrosContext
extern std::mutex g_type_adapter_nitros_context_mutex;

// Is the global type adapter's global NitrosContext initialized?
extern bool g_type_adapter_nitros_context_initialized;

// Is the global type adapter's global NitrosContext destroyed?
extern bool g_type_adapter_nitros_context_destroyed;

// Get the type adapter's global NitrosContext object
// If not initialized, the first call creates an NitrosContext object and starts
// the type adapter's graph
NitrosContext & GetTypeAdapterNitrosContext();

// Terminate the type adapter's graph and release the context
void DestroyTypeAdapterNitrosContext();

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__TYPE_ADAPTER_NITROS_CONTEXT_HPP_
