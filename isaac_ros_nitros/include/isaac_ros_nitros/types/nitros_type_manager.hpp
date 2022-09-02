/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MANAGER_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MANAGER_HPP_

#include <map>
#include <string>
#include <vector>
#include <utility>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosTypeManager
{
public:
  // Constructor
  NitrosTypeManager() = default;
  explicit NitrosTypeManager(const rclcpp::Node * node)
  {
    setNode(node);
  }

  // Setter for node_ that is used to get ROS logger if set
  void setNode(const rclcpp::Node * node)
  {
    node_ = node;
  }

  // Register supported data formats from the given data type
  template<typename T>
  void registerSupportedType()
  {
    auto type_name = typeid(T).name();

    if (type_format_map_.count(type_name) != 0) {
      RCLCPP_INFO(
        get_logger(),
        "[NitrosTypeManager] Skipped registering duplicate type \"%s\"",
        type_name);
      return;
    }

    // Register format callbacks
    auto type_format_callback_map = T::GetFormatCallbacks();
    format_callback_map_.insert(
      type_format_callback_map.begin(), type_format_callback_map.end());

    // Register formats to the type
    if (type_format_map_.count(type_name) == 0) {
      type_format_map_.insert({type_name, {}});
    }
    for (auto const & it : type_format_callback_map) {
      type_format_map_.at(type_name).push_back(it.first);
    }

    // Register the type's required extensions
    auto type_extension_list = T::GetExtensions();
    type_extension_map_.insert({type_name, type_extension_list});
  }

  // Get all extension paths for all registered types
  std::vector<std::pair<std::string, std::string>> getExtensions() const
  {
    std::vector<std::pair<std::string, std::string>> extensions;
    for (const auto & extension_pairs : type_extension_map_) {
      extensions.insert(
        extensions.end(),
        extension_pairs.second.begin(),
        extension_pairs.second.end());
    }
    return extensions;
  }

  // Check if the given format name has been registered
  bool hasFormat(const std::string format_name) const
  {
    return format_callback_map_.count(format_name) > 0;
  }

  // Get a data format's callback functions
  const NitrosFormatCallbacks & getFormatCallbacks(const std::string format_name) const
  {
    return format_callback_map_.at(format_name);
  }

  // Get a list of data format strings of all the registered data types
  std::vector<std::string> getAllRegisteredDataFormats()
  {
    std::vector<std::string> format_list;
    for (const auto & format_callback : format_callback_map_) {
      format_list.push_back(format_callback.first);
    }
    return format_list;
  }

private:
  // Get the manager's logger if set. Forward to rclcpp::get_logger() otherwise.
  rclcpp::Logger get_logger()
  {
    if (node_ != nullptr) {
      return node_->get_logger();
    }
    return rclcpp::get_logger("NitrosTypeManager");
  }

  // The associated ROS node (for logging purpose)
  const rclcpp::Node * node_ = nullptr;

  // A map storing callback functions for each registered format
  std::map<std::string, NitrosFormatCallbacks> format_callback_map_;

  // A map for a type's formats
  std::map<std::string, std::vector<std::string>> type_format_map_;

  // A map for a type's list of required component extension so files
  std::map<std::string, std::vector<std::pair<std::string, std::string>>>
  type_extension_map_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MANAGER_HPP_
