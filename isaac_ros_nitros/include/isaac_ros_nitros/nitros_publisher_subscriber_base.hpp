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

#ifndef ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
#define ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_

#include <algorithm>
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

  // Enable NitrosNode to adjust the compatible format after an unsuccessful
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

  // Map of topic name and corresponding expected time difference between messages in microseconds
  std::map<std::string, int> topic_name_expected_dt_map;

  // Tolerance for jitter from expected frame rate in microseconds
  int jitter_tolerance_us{5000};
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

  const NitrosPublisherSubscriberConfig getConfig() const
  {
    return config_;
  }

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
    } else {
      RCLCPP_FATAL(
        node_.get_logger(),
        "[NitrosPublisherSubscriberBase] Failed to resolve entity (eid=%ld) (error=%s)",
        base_msg.handle, GxfResultStr(msg_entity.error()));
      return 0;
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

  void throwOnUnsupportedCompatibleDataFormat()
  {
    if (std::find(
        supported_data_formats_.begin(),
        supported_data_formats_.end(),
        config_.compatible_data_format) == supported_data_formats_.end())
    {
      std::stringstream error_msg;
      error_msg <<
        "[NitrosPublisherSubscriberBase] Specified compatible format " <<
        "\"" << config_.compatible_data_format.c_str() << "\" was not listed in " <<
        "the supported format list: [";
      for (size_t i = 0; i < supported_data_formats_.size(); i++) {
        if (i > 0) {
          error_msg << ", ";
        }
        error_msg << supported_data_formats_[i].c_str();
      }
      error_msg << "]";
      RCLCPP_ERROR(
        node_.get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // Update statistics numbers. To be called in nitros Subscriber and Publisher
  void updateStatistics()
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateStatistics() and publishNitrosStatistics()
    const std::lock_guard<std::mutex> lock(nitros_statistics_mutex_);
    // NITROS statistics
    std::chrono::time_point<std::chrono::steady_clock> current_timestamp = clock_.now();

    // we can only calculate frame rate after 2 messages have been received
    if (prev_msg_timestamp_ != std::chrono::steady_clock::time_point::min()) {
      int microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        current_timestamp - prev_msg_timestamp_).count();
      // calculate difference between time between msgs(using system clock)
      // and expected time between msgs
      int abs_jitter = std::abs(
        microseconds -
        statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name]);
      if (abs_jitter > statistics_config_.jitter_tolerance_us) {
        RCLCPP_WARN(
          node_.get_logger(),
          "[NitrosStatistics] Difference of time between messages(%i) and expected time between"
          " messages(%i) is out of tolerance(%i) by %i for topic %s. Units are microseconds.",
          microseconds, statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name],
          statistics_config_.jitter_tolerance_us, abs_jitter, statistics_msg_.topic_name.c_str());
      }
      // Update max abs jitter
      max_abs_jitter_ = std::max(max_abs_jitter_, abs_jitter);
      msg_jitter_queue_.push(abs_jitter);
      msg_interarrival_time_queue_.push(microseconds);

      // add and subtract weighted datapoints to prevent summation of entire queue at
      // each run of this function. Also, support mean filter when the filter size
      // has not reached the maximum limit of config_.filter_window_size
      sum_msg_jitter_ += abs_jitter;
      sum_msg_interarrival_time_ += microseconds;
      if (static_cast<int>(msg_interarrival_time_queue_.size()) >
        statistics_config_.filter_window_size)
      {
        sum_msg_jitter_ -= msg_jitter_queue_.front();
        sum_msg_interarrival_time_ -= msg_interarrival_time_queue_.front();
        msg_jitter_queue_.pop();
        msg_interarrival_time_queue_.pop();
      }
    }
    prev_msg_timestamp_ = current_timestamp;
  }

  // Function to publish statistics to a ROS topic
  void publishNitrosStatistics()
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateStatistics() and publishNitrosStatistics()
    const std::lock_guard<std::mutex> lock(nitros_statistics_mutex_);
    // publish zero until atleast one message has been received
    if (sum_msg_interarrival_time_ != 0) {
      statistics_msg_.frame_rate = kMicrosecondsInSeconds /
        (static_cast<float>(sum_msg_interarrival_time_) / msg_interarrival_time_queue_.size());
      statistics_msg_.mean_abs_jitter = sum_msg_jitter_ / msg_jitter_queue_.size();
    } else {
      statistics_msg_.frame_rate = 0.0;
      statistics_msg_.mean_abs_jitter = 0;
    }
    statistics_msg_.max_abs_jitter = max_abs_jitter_;
    statistics_publisher_->publish(statistics_msg_);
  }

  // Initialize statistics variables
  void initStatistics()
  {
    statistics_msg_.node_name = node_.get_name();
    statistics_msg_.node_namespace = node_.get_namespace();
    statistics_msg_.topic_name = config_.topic_name;

    // Check if expected topic name is present in topic_name_expected_dt_map
    if (!statistics_config_.topic_name_expected_dt_map.count(statistics_msg_.topic_name)) {
      std::stringstream error_msg;
      error_msg << "[NitrosNode]" << statistics_msg_.topic_name <<
        " topic name not found in topics_list ROS param";
      RCLCPP_ERROR(node_.get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }

    // Initialize varibles to min and zero as a flag to detect no messages have been received
    prev_msg_timestamp_ = std::chrono::steady_clock::time_point::min();
    sum_msg_interarrival_time_ = 0;
    sum_msg_jitter_ = 0;
    max_abs_jitter_ = 0;

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
  gxf_context_t context_ = nullptr;

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

  // Mutex lock to prevent simultaneous access of common parameters
  // used by updateStatistics() and publishNitrosStatistics()
  std::mutex nitros_statistics_mutex_;

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

  // Queue to store message jitter to implement windowed mean filter
  // Jitter is the difference between the time between msgs(dt)
  // calculated from fps specified in NITROS statistics ROS param
  // and measured using system clock
  std::queue<int> msg_jitter_queue_;

  // Sum of the message interarrival times received on this topic within the configured window
  int64_t sum_msg_interarrival_time_;

  // Sum of the message jitter on this topic within the configured window
  int64_t sum_msg_jitter_;

  // Max absolute jitter
  int max_abs_jitter_;

  // Prev timestamp stored for calculating frame rate
  std::chrono::time_point<std::chrono::steady_clock> prev_msg_timestamp_;

  // NITROS statistics publisher timer
  rclcpp::TimerBase::SharedPtr statistics_publisher_timer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
