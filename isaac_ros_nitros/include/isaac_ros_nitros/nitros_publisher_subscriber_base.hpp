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

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "extensions/gxf_optimizer/exporter/graph_types.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/timestamp.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/nitros_type_manager.hpp"

#include "isaac_ros_nitros_interfaces/msg/topic_statistics.hpp"
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

namespace
{
constexpr float const kMicrosecondsInSeconds = 1000000;
}

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

// Configurations for a Nitros statistics
struct NitrosStatisticsConfig
{
  // Statistics toggle
  bool enable_statistics{false};

  // Rate (Hz) at which to publish statistics to a ROS topic
  float statistics_publish_rate{1.0};

  // Window size of the mean filter in terms of number of messages received
  int filter_window_size{100};
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
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config)
  : node_(node),
    nitros_type_manager_(nitros_type_manager),
    gxf_component_info_(gxf_component_info),
    supported_data_formats_(supported_data_formats),
    config_(config) {}

  NitrosPublisherSubscriberBase(
    rclcpp::Node & node,
    const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config)
  : node_(node),
    context_(context),
    nitros_type_manager_(nitros_type_manager),
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

  uint64_t getTimestamp(NitrosTypeBase & base_msg) const
  {
    auto msg_entity = nvidia::gxf::Entity::Shared(context_, base_msg.handle);
    if (msg_entity) {
      auto timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
      if (timestamp) {
        return timestamp.value()->acqtime;
      }
    }
    RCLCPP_WARN(
      node_.get_logger(),
      "[NitrosPublisherSubscriberBase] Failed to get timestamp from a NITROS"
      " message (eid=%ld)",
      base_msg.handle);
    return 0;
  }

  // Start negotiation
  virtual void start() = 0;

  // To be called after negotiation timer is up
  virtual void postNegotiationCallback() = 0;

  // Update statistics numbers. To be called in nitros Subscriber and Publisher
  void updateStatistics()
  {
    // Mutex lock to prevent simultaneous access of sum_msg_interarrival_time_
    const std::lock_guard<std::mutex> lock(sum_msg_interarrival_time_mutex_);
    // NITROS statistics
    std::chrono::time_point<std::chrono::steady_clock> current_timestamp = clock_.now();

    // we can only calculate frame rate after 2 messages have been received
    if (prev_msg_timestamp_ != std::chrono::steady_clock::time_point::min()) {
      int microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        current_timestamp - prev_msg_timestamp_).count();
      msg_interarrival_time_queue_.push(microseconds);

      // add and subtract weighted datapoints to prevent summation of entire queue at
      // each run of this function. Also, support mean filter when the filter size
      // has not reached the maximum limit of config_.filter_window_size
      sum_msg_interarrival_time_ += microseconds;
      if (static_cast<int>(msg_interarrival_time_queue_.size()) >
        statistics_config_.filter_window_size)
      {
        sum_msg_interarrival_time_ -= msg_interarrival_time_queue_.front();
        msg_interarrival_time_queue_.pop();
      }
    }
    prev_msg_timestamp_ = current_timestamp;
  }

  // Function to publish statistics to a ROS topic
  void publishNitrosStatistics()
  {
    // Mutex lock to prevent simultaneous access of sum_msg_interarrival_time_
    const std::lock_guard<std::mutex> lock(sum_msg_interarrival_time_mutex_);
    // publish zero until atleast one message has been received
    if (sum_msg_interarrival_time_ != 0) {
      statistics_msg_.frame_rate = kMicrosecondsInSeconds /
        (static_cast<float>(sum_msg_interarrival_time_) / msg_interarrival_time_queue_.size());
    } else {
      statistics_msg_.frame_rate = 0.0;
    }
    statistics_publisher_->publish(statistics_msg_);
  }

  // Initialize statistics variables
  void initStatistics()
  {
    statistics_msg_.node_name = node_.get_name();
    statistics_msg_.node_namespace = node_.get_namespace();
    statistics_msg_.topic_name = config_.topic_name;

    // Initialize varibles to min and zero as a flag to detect no messages have been received
    prev_msg_timestamp_ = std::chrono::steady_clock::time_point::min();
    sum_msg_interarrival_time_ = 0;

    // Initialize statistics publisher and start a timer callback to publish at a fixed rate
    statistics_publisher_ =
      node_.create_publisher<isaac_ros_nitros_interfaces::msg::TopicStatistics>(
      "/nitros_statistics", 10);
    statistics_publisher_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int>(1000 / statistics_config_.statistics_publish_rate)),
      [this]() -> void {
        publishNitrosStatistics();
      });
  }

protected:
  // The parent ROS 2 node
  rclcpp::Node & node_;

  // The parent GXF context
  gxf_context_t context_;

  // Nitros type manager
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;

  // The info of the GXF component that's associated to this Nitros publisher/subscriber
  gxf::optimizer::ComponentInfo gxf_component_info_;

  // Supported data formats
  std::vector<std::string> supported_data_formats_;

  // Configurations for creating the corresponding publisher/subscriber
  NitrosPublisherSubscriberConfig config_;

  // Negotiated data format
  std::string negotiated_data_format_;

  // Frame ID map
  std::shared_ptr<std::map<gxf::optimizer::ComponentKey, std::string>> frame_id_map_ptr_;

  // Mutex to prevent simultaneous access
  std::mutex sum_msg_interarrival_time_mutex_;

  // NITROS statistics variables
  // Configurations for a Nitros statistics
  NitrosStatisticsConfig statistics_config_;

  // Statistics Data publisher
  rclcpp::Publisher<isaac_ros_nitros_interfaces::msg::TopicStatistics>::SharedPtr
    statistics_publisher_;

  // Satistics ROS msg
  isaac_ros_nitros_interfaces::msg::TopicStatistics statistics_msg_;

  // Clock object used to retrived current timestamps
  std::chrono::steady_clock clock_;

  // Queue to store time between messages to implement windowed mean filter
  std::queue<int> msg_interarrival_time_queue_;

  // Sum of the message interarrival times received on this topic within the configured window
  int64_t sum_msg_interarrival_time_;

  // Prev timestamp stored for calculating frame rate
  std::chrono::time_point<std::chrono::steady_clock> prev_msg_timestamp_;

  // NITROS statistics publisher timer
  rclcpp::TimerBase::SharedPtr statistics_publisher_timer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
