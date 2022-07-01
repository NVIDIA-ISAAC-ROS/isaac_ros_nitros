/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
#define ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "extensions/gxf_optimizer/exporter/graph_types.hpp"
#include "gxf/core/gxf.h"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/rclcpp.hpp"

#if defined(USE_NVTX)
  #include "nvToolsExt.h"
#endif


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Enum for specifying vairous types of Nitros publishers/subscribers
enum NitrosPublisherSubscriberType
{
  NOOP = 0,
  NON_NEGOTIATED,
  NEGOTIATED,
};

// Configurations for a Nitros publisher/subscriber
struct NitrosPublisherSubscriberConfig
{
  NitrosPublisherSubscriberType type{NitrosPublisherSubscriberType::NOOP};
  rclcpp::QoS qos{rclcpp::QoS(1)};
  std::string compatible_data_format{""};
  std::string topic_name{""};

  // Pin this pub/sub to use only the compatible format for negotiation
  bool use_compatible_format_only{false};

  // Eanble NitrosNode to adjust the compatible format after an unsuccessful
  // negotiation. The compatible format is adjusted if the existing compatible
  // format, together with other selected data formats in the same pub/sub group,
  // cannot form a valid data format combination.
  bool use_flexible_compatible_format{true};

  // ComponentKey whose frame_id this pub/sub should refer to
  std::string frame_id_source_key{""};

  // User-defined callback function
  std::function<void(const gxf_context_t, NitrosTypeBase &)> callback{nullptr};
};

using NitrosPublisherSubscriberConfigMap =
  std::map<gxf::optimizer::ComponentKey, NitrosPublisherSubscriberConfig>;

// Nitros publisher/subscriber base class
class NitrosPublisherSubscriberBase
{
public:
  // Constructor
  NitrosPublisherSubscriberBase(
    rclcpp::Node & node,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config)
  : node_(node),
    gxf_component_info_(gxf_component_info),
    supported_data_formats_(supported_data_formats),
    config_(config) {}

  // Getter for the GXF component info
  gxf::optimizer::ComponentInfo getComponentInfo()
  {
    return gxf_component_info_;
  }

  // Getter for the negotiated data format
  std::string getNegotiatedDataFormat() const
  {
    return negotiated_data_format_;
  }

  // Getter for the compatible data format
  std::string getCompatibleDataFormat() const
  {
    return config_.compatible_data_format;
  }

  // Setter for the compatible data format
  void setCompatibleDataFormat(const std::string & compatible_data_format)
  {
    config_.compatible_data_format = compatible_data_format;
  }

  // Get the negotiated data format or the compatible format if negotiation failed
  std::string getFinalDataFormat() const
  {
    if (negotiated_data_format_.empty()) {
      return config_.compatible_data_format;
    }
    return negotiated_data_format_;
  }

  // Getter for the GXF context
  gxf_context_t getContext()
  {
    return context_;
  }

  // Setter for the parent GXF context
  void setContext(const gxf_context_t context)
  {
    context_ = context;
  }

  // Setter for the frame_id passthrough map
  void setFrameIdMap(
    std::shared_ptr<std::map<gxf::optimizer::ComponentKey, std::string>> frame_id_map_ptr)
  {
    frame_id_map_ptr_ = frame_id_map_ptr;
  }

  // Start negotiation
  virtual void start() = 0;

  // To be called after negotiation timer is up
  virtual void postNegotiationCallback() = 0;

protected:
  // The parent ROS2 node
  rclcpp::Node & node_;

  // The parent GXF context
  gxf_context_t context_;

  // The info of the GXF component that's associated to this Nitros publisher/subscriber
  gxf::optimizer::ComponentInfo gxf_component_info_;

  // Supported data formats
  std::vector<std::string> supported_data_formats_;

  // Configurations for creating the corresponding publisher/subscriber
  NitrosPublisherSubscriberConfig config_;

  // Negotiated data format
  std::string negotiated_data_format_;

  std::shared_ptr<std::map<gxf::optimizer::ComponentKey, std::string>> frame_id_map_ptr_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
