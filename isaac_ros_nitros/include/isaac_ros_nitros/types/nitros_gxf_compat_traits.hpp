// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_GXF_COMPAT_TRAITS_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_GXF_COMPAT_TRAITS_HPP_

#ifdef NITROS_GXF_COMPAT_MODE

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeindex>

#include "gxf/core/gxf.h"
#include "rclcpp/rclcpp.hpp"
#include "negotiated/negotiated_publisher.hpp"

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/nitros_format_agent.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// ============================================================================
// INTERIM: GXF Compatibility Framework for GXF-free NITROS Types
// TODO(yuankunz): Remove when all nodes migrated to GXF-free implementation
//
// To add GXF compatibility for a GXF-free NITROS type:
//
// Specialize NitrosGxfCompatTraits<YourType> with 2 functions:
//   - CreateGxfEntity: YourType → GXF entity
//   - CreateFromGxfEntity: GXF entity → YourType
// ============================================================================

template<typename MessageType>
struct NitrosGxfCompatTraits
{
  static int64_t CreateGxfEntity(
    gxf_context_t context,
    const MessageType & msg);

  static MessageType CreateFromGxfEntity(
    gxf_context_t context,
    int64_t eid);
};

template<typename T, typename = void>
struct has_nitros_gxf_compat : std::false_type {};

template<typename T>
struct has_nitros_gxf_compat<T, std::void_t<
    decltype(NitrosGxfCompatTraits<T>::CreateGxfEntity(
    std::declval<gxf_context_t>(),
    std::declval<const T &>()))
  >>: std::true_type {};

// ============================================================================
// Conversion Registry (for runtime dispatch)
// ============================================================================
//
// The registry enables type-erased conversion from GXF-free NITROS messages
// to GXF entities at runtime. This is needed when:
//   1. A new GXF-free node to a legacy NITROS node subscriber
//   2. The subscriber receives NitrosTypeBase& (type-erased) and needs to
//      convert it to a GXF entity without knowing the concrete type at compile time
//
// Each GXF-free type with GXF compat registers its converter at static init time
// using NITROS_GXF_COMPAT_REGISTER_CONVERTER macro.

using NitrosToGxfConverter = std::function<int64_t(
      gxf_context_t,
      NitrosTypeBase &)>;

inline std::map<std::type_index, NitrosToGxfConverter> & GetNitrosGxfConverterRegistry()
{
  static std::map<std::type_index, NitrosToGxfConverter> registry;
  return registry;
}

template<typename MessageType>
void RegisterNitrosGxfConverter()
{
  auto & registry = GetNitrosGxfConverterRegistry();
  registry[std::type_index(typeid(MessageType))] =
    [](gxf_context_t ctx, NitrosTypeBase & base_msg) -> int64_t {
      auto & typed_msg = static_cast<MessageType &>(base_msg);
      return NitrosGxfCompatTraits<MessageType>::CreateGxfEntity(ctx, typed_msg);
    };
}

inline int64_t CreateGxfEntityFromNitros(
  gxf_context_t context,
  NitrosTypeBase & msg_base)
{
  auto & registry = GetNitrosGxfConverterRegistry();
  auto type_idx = std::type_index(typeid(msg_base));
  auto it = registry.find(type_idx);
  if (it != registry.end()) {
    return it->second(context, msg_base);
  }

  RCLCPP_ERROR(rclcpp::get_logger("NitrosGxfCompat"),
    "No GXF converter registered for type: %s (registry size=%zu)",
    type_idx.name(), registry.size());
  return -1;
}

// ============================================================================
// Callback Wrappers for GXF-Compatible Publishing
// ============================================================================
//
// These callbacks enable legacy GXF nodes to publish GXF-free NITROS messages.
// When a legacy GXF node outputs a GXF entity (handle>=0), these callbacks:
//   1. Convert GXF entity → GXF-free NITROS message (zero-copy)
//   2. Publish the GXF-free message to ROS2 topics

template<typename FormatType>
static NitrosFormatCallbacks MakeGxfCompatCallbacks(NitrosFormatCallbacks base)
{
  using MessageType = typename FormatType::MsgT;
  auto context = GetTypeAdapterNitrosContext().getContext();

  // Negotiated publish: Publishes to negotiated topic with format type information
  base.negotiatedPublishCallback = [context](
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedPublisher> pub,
    NitrosTypeBase & base_msg) {
      (void)node;
      if (base_msg.handle >= 0) {
        // GXF-based message: convert from GXF entity
        MessageType msg = NitrosGxfCompatTraits<MessageType>::CreateFromGxfEntity(
          context, base_msg.handle);
        pub->template publish<FormatType>(msg);
      } else {
        // Buffer-based message: publish reference directly (no copy assignment)
        pub->template publish<FormatType>(static_cast<MessageType &>(base_msg));
      }
    };

  // Compatible publish: Publishes to standard topic as fallback
  base.compatiblePublishCallback = [context](
    rclcpp::Node & node,
    std::shared_ptr<rclcpp::PublisherBase> pub,
    NitrosTypeBase & base_msg) {
      (void)node;
      auto cast_pub = static_cast<rclcpp::Publisher<MessageType> *>(pub.get());
      if (base_msg.handle >= 0) {
        // GXF-based message: convert from GXF entity
        MessageType msg = NitrosGxfCompatTraits<MessageType>::CreateFromGxfEntity(
          context, base_msg.handle);
        cast_pub->publish(msg);
      } else {
        // Buffer-based message: publish reference directly (no copy assignment)
        cast_pub->publish(static_cast<MessageType &>(base_msg));
      }
    };

  return base;
}

// ============================================================================
// Convenience Macros
// ============================================================================

#define NITROS_GXF_COMPAT_FORMATS_BEGIN(TYPE_NAME) \
  std::map<std::string, NitrosFormatCallbacks> TYPE_NAME::GetFormatCallbacks() \
  { \
    std::map<std::string, NitrosFormatCallbacks> format_callback_map;

#define NITROS_GXF_COMPAT_FORMAT_ADD(FORMAT_TYPE) \
  format_callback_map.emplace( \
    FORMAT_TYPE::supported_type_name, \
    MakeGxfCompatCallbacks<FORMAT_TYPE>( \
      NitrosFormatAgent<FORMAT_TYPE>::GetFormatCallbacks()));

#define NITROS_GXF_COMPAT_FORMATS_END() \
  return format_callback_map; \
  }

#define NITROS_GXF_COMPAT_REGISTER_CONVERTER(TYPE_NAME) \
  namespace { \
  struct TYPE_NAME ## GxfConverterRegistrar { \
    TYPE_NAME ## GxfConverterRegistrar() { \
      RegisterNitrosGxfConverter<TYPE_NAME>(); \
    } \
  }; \
  static TYPE_NAME ## GxfConverterRegistrar g_ ## TYPE_NAME ## _gxf_registrar; \
  }

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_GXF_COMPAT_TRAITS_HPP_
