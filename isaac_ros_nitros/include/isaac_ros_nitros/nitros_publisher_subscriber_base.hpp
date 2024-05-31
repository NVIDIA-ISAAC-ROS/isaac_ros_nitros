// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <limits>
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
constexpr float const kSecondsToMicroseconds = 1000000;
constexpr uint64_t const kMicrosecondsToNanoseconds = 1000;
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
  bool enable_all_statistics{false};
  bool enable_node_time_statistics{false};
  bool enable_msg_time_statistics{false};
  bool enable_increasing_msg_time_statistics{false};

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

    RCLCPP_DEBUG(
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
  void updateStatistics(uint64_t msg_timestamp_ns)
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateStatistics() and publishNitrosStatistics()
    const std::lock_guard<std::mutex> lock(nitros_statistics_mutex_);
    // NITROS statistics checks message intervals both using the node clock
    // and the message timestamp.
    // All variables name _node refers to the node timestamp checks.
    // All variables name _msg refers to the message timestamp checks.
    std::chrono::time_point<std::chrono::steady_clock> current_timestamp_node = clock_.now();
    // Convert nanoseconds to microseconds
    uint64_t current_timestamp_msg_us = msg_timestamp_ns / kMicrosecondsToNanoseconds;

    // we can only calculate frame rate after 2 messages have been received
    if (prev_timestamp_node_ != std::chrono::steady_clock::time_point::min() &&
      statistics_config_.enable_node_time_statistics)
    {
      int microseconds_node = std::chrono::duration_cast<std::chrono::microseconds>(
        current_timestamp_node - prev_timestamp_node_).count();
      // calculate difference between time between msgs(using node clock)
      // and expected time between msgs
      int abs_jitter_node = std::abs(
        microseconds_node -
        statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name]);
      if (abs_jitter_node > statistics_config_.jitter_tolerance_us) {
        // Increment jitter outlier count
        num_jitter_outliers_node_++;
        RCLCPP_WARN(
          node_.get_logger(),
          "[NitrosStatistics Node Time]"
          " Difference of time between messages(%i) and expected time between"
          " messages(%i) is out of tolerance(%i) by %i for topic %s. Units are microseconds.",
          microseconds_node,
          statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name],
          statistics_config_.jitter_tolerance_us,
          abs_jitter_node, statistics_msg_.topic_name.c_str());
      }
      // Update max abs jitter
      max_abs_jitter_node_ = std::max(max_abs_jitter_node_, abs_jitter_node);
      jitter_queue_node_.push(abs_jitter_node);
      interarrival_time_queue_node_.push(microseconds_node);

      // add and subtract weighted datapoints to prevent summation of entire queue at
      // each run of this function. Also, support mean filter when the filter size
      // has not reached the maximum limit of config_.filter_window_size
      sum_jitter_node_ += abs_jitter_node;
      sum_interarrival_time_node_ += microseconds_node;
      if (static_cast<int>(interarrival_time_queue_node_.size()) >
        statistics_config_.filter_window_size)
      {
        sum_jitter_node_ -= jitter_queue_node_.front();
        sum_interarrival_time_node_ -= interarrival_time_queue_node_.front();
        jitter_queue_node_.pop();
        interarrival_time_queue_node_.pop();
      }
    }

    // Do the same checks as above, but for message timestamp
    if (prev_timestamp_msg_ != std::numeric_limits<uint64_t>::min() &&
      statistics_config_.enable_msg_time_statistics)
    {
      uint64_t microseconds_msg = current_timestamp_msg_us - prev_timestamp_msg_;
      // calculate difference between time between msgs(using msgtem clock)
      // and expected time between msgs
      int abs_jitter_msg = std::abs(
        static_cast<int>(microseconds_msg) -
        statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name]);
      if (abs_jitter_msg > statistics_config_.jitter_tolerance_us) {
        // Increment jitter outlier count
        num_jitter_outliers_msg_++;
        RCLCPP_WARN(
          node_.get_logger(),
          "[NitrosStatistics Message Timestamp]"
          " Difference of time between messages(%lu) and expected time between"
          " messages(%i) is out of tolerance(%i) by %i for topic %s. Units are microseconds.",
          microseconds_msg,
          statistics_config_.topic_name_expected_dt_map[statistics_msg_.topic_name],
          statistics_config_.jitter_tolerance_us,
          abs_jitter_msg, statistics_msg_.topic_name.c_str());
      }
      // Update max abs jitter
      max_abs_jitter_msg_ = std::max(max_abs_jitter_msg_, abs_jitter_msg);
      jitter_queue_msg_.push(abs_jitter_msg);
      interarrival_time_queue_msg_.push(microseconds_msg);

      // add and subtract weighted datapoints to prevent summation of entire queue at
      // each run of this function. Also, support mean filter when the filter size
      // has not reached the maximum limit of config_.filter_window_size
      sum_jitter_msg_ += abs_jitter_msg;
      sum_interarrival_time_msg_ += microseconds_msg;
      if (static_cast<int>(interarrival_time_queue_msg_.size()) >
        statistics_config_.filter_window_size)
      {
        sum_jitter_msg_ -= jitter_queue_msg_.front();
        sum_interarrival_time_msg_ -= interarrival_time_queue_msg_.front();
        jitter_queue_msg_.pop();
        interarrival_time_queue_msg_.pop();
      }
    }

    if (prev_timestamp_msg_ != std::numeric_limits<uint64_t>::min() &&
      statistics_config_.enable_increasing_msg_time_statistics)
    {
      // Check if message timestamp is increasing
      if (current_timestamp_msg_us < prev_timestamp_msg_) {
        // Increment non increasing message count
        num_non_increasing_msg_++;
        RCLCPP_WARN(
          node_.get_logger(),
          "[NitrosStatistics Message Timestamp Non Increasing]"
          " Message timestamp is not increasing. Current timestamp: %lu, Previous timestamp: %lu"
          " for topic %s. Units are microseconds.",
          current_timestamp_msg_us, prev_timestamp_msg_, statistics_msg_.topic_name.c_str());
      }
    }

    prev_timestamp_node_ = current_timestamp_node;
    prev_timestamp_msg_ = current_timestamp_msg_us;
  }

  // Function to publish statistics to a ROS topic
  void publishNitrosStatistics()
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateStatistics() and publishNitrosStatistics()
    const std::lock_guard<std::mutex> lock(nitros_statistics_mutex_);
    // publish zero until atleast one message has been received
    if (sum_interarrival_time_node_ != 0) {
      statistics_msg_.frame_rate_node = kSecondsToMicroseconds /
        (static_cast<float>(sum_interarrival_time_node_) / interarrival_time_queue_node_.size());
      statistics_msg_.mean_abs_jitter_node = sum_jitter_node_ / jitter_queue_node_.size();
      statistics_msg_.num_jitter_outliers_node = num_jitter_outliers_node_;
    } else {
      statistics_msg_.frame_rate_node = 0.0;
      statistics_msg_.mean_abs_jitter_node = 0;
      statistics_msg_.num_jitter_outliers_node = 0;
    }
    if (sum_interarrival_time_msg_ != 0) {
      statistics_msg_.frame_rate_msg = kSecondsToMicroseconds /
        (static_cast<float>(sum_interarrival_time_msg_) / interarrival_time_queue_msg_.size());
      statistics_msg_.mean_abs_jitter_msg = sum_jitter_msg_ / jitter_queue_msg_.size();
      statistics_msg_.num_jitter_outliers_msg = num_jitter_outliers_msg_;
    } else {
      statistics_msg_.frame_rate_msg = 0.0;
      statistics_msg_.mean_abs_jitter_msg = 0;
      statistics_msg_.num_jitter_outliers_msg = 0;
    }
    statistics_msg_.max_abs_jitter_node = max_abs_jitter_node_;
    statistics_msg_.max_abs_jitter_msg = max_abs_jitter_msg_;
    statistics_msg_.num_non_increasing_msg = num_non_increasing_msg_;
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
    prev_timestamp_node_ = std::chrono::steady_clock::time_point::min();
    prev_timestamp_msg_ = std::numeric_limits<uint64_t>::min();
    sum_interarrival_time_node_ = 0;
    sum_interarrival_time_msg_ = 0;
    sum_jitter_node_ = 0;
    sum_jitter_msg_ = 0;
    max_abs_jitter_node_ = 0;
    max_abs_jitter_msg_ = 0;
    num_jitter_outliers_node_ = 0;
    num_jitter_outliers_msg_ = 0;
    num_non_increasing_msg_ = 0;

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
  std::queue<int> interarrival_time_queue_node_;
  std::queue<uint64_t> interarrival_time_queue_msg_;

  // Queue to store message jitter to implement windowed mean filter
  // Jitter is the difference between the time between msgs(dt)
  // calculated from fps specified in NITROS statistics ROS param
  // and measured using node clock
  std::queue<int> jitter_queue_node_;
  std::queue<int> jitter_queue_msg_;

  // Sum of the message interarrival times received on this topic within the configured window
  int64_t sum_interarrival_time_node_;
  uint64_t sum_interarrival_time_msg_;

  // Sum of the message jitter on this topic within the configured window
  int64_t sum_jitter_node_;
  int sum_jitter_msg_;

  // Max absolute jitter
  int max_abs_jitter_node_;
  int max_abs_jitter_msg_;

  // Number of messages outside the jitter tolerance
  uint64_t num_jitter_outliers_node_;
  uint64_t num_jitter_outliers_msg_;

  // Number of non-increasing messages
  uint64_t num_non_increasing_msg_;

  // Prev timestamp stored for calculating frame rate
  // Prev node timstamp
  std::chrono::time_point<std::chrono::steady_clock> prev_timestamp_node_;
  // Prev message timestamp in microseconds
  uint64_t prev_timestamp_msg_;

  // NITROS statistics publisher timer
  rclcpp::TimerBase::SharedPtr statistics_publisher_timer_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
