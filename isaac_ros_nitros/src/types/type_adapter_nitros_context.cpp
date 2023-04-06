/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char TYPE_ADAPTER_CONTEXT_YAML[] =
  "config/type_adapter_nitros_context_graph.yaml";

const std::vector<std::pair<std::string, std::string>> TYPE_ADAPTER_EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_gxf_helpers.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_sight.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_atlas.so"}
};

std::unique_ptr<NitrosContext> g_type_adapter_nitros_context;
std::mutex g_type_adapter_nitros_context_mutex;
bool g_type_adapter_nitros_context_initialized = false;
bool g_type_adapter_nitros_context_destroyed = true;

NitrosContext & GetTypeAdapterNitrosContext()
{
  // Mutex: g_type_adapter_nitros_context_mutex
  const std::lock_guard<std::mutex> lock(g_type_adapter_nitros_context_mutex);
  gxf_result_t code;
  if (g_type_adapter_nitros_context_initialized == false) {
    g_type_adapter_nitros_context = std::make_unique<NitrosContext>();
    const std::string nitros_package_share_directory =
      ament_index_cpp::get_package_share_directory("isaac_ros_nitros");

    // Load extensions
    for (const auto & extension_pair : TYPE_ADAPTER_EXTENSIONS) {
      const std::string package_directory =
        ament_index_cpp::get_package_share_directory(extension_pair.first);
      code = g_type_adapter_nitros_context->loadExtension(package_directory, extension_pair.second);
      if (code != GXF_SUCCESS) {
        std::stringstream error_msg;
        error_msg << "loadExtensions Error: " << GxfResultStr(code);
        RCLCPP_ERROR(rclcpp::get_logger("TypeAdapterNitrosContext"), error_msg.str().c_str());
        throw std::runtime_error(error_msg.str().c_str());
      }
    }

    // Load application
    code = g_type_adapter_nitros_context->loadApplication(
      nitros_package_share_directory + "/" + TYPE_ADAPTER_CONTEXT_YAML);
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "loadApplication Error: " << GxfResultStr(code);
      RCLCPP_ERROR(rclcpp::get_logger("TypeAdapterNitrosContext"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    // Run graph
    code = g_type_adapter_nitros_context->runGraphAsync();
    if (code != GXF_SUCCESS) {
      std::stringstream error_msg;
      error_msg << "runGraphAsync Error: " << GxfResultStr(code);
      RCLCPP_ERROR(rclcpp::get_logger("TypeAdapterNitrosContext"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }


    g_type_adapter_nitros_context_initialized = true;
    g_type_adapter_nitros_context_destroyed = false;
  }
  return *g_type_adapter_nitros_context;
  // End Mutex: g_type_adapter_nitros_context_mutex
}

void DestroyTypeAdapterNitrosContext()
{
  const std::lock_guard<std::mutex> lock(g_type_adapter_nitros_context_mutex);
  if (!g_type_adapter_nitros_context_destroyed) {
    g_type_adapter_nitros_context->destroy();
    g_type_adapter_nitros_context.reset();
    g_type_adapter_nitros_context_destroyed = true;
    g_type_adapter_nitros_context_initialized = false;
  }
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
