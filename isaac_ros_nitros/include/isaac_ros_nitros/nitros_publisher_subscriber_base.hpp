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

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
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
const char * const nvidiaID = "nvidia";
constexpr int64_t const kDropWarnTimeoutSeconds = 5;
}  // namespace

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
  // format, together with other selected data formats in the same pub/sub
  // group, cannot form a valid data format combination.
  bool use_flexible_compatible_format{true};

  // ComponentKey whose frame_id this pub/sub should refer to
  std::string frame_id_source_key{""};

  // User-defined callback function
  std::function<void(const gxf_context_t, NitrosTypeBase &)> callback{nullptr};
};

// Configurations for a Nitros diagnostics
struct NitrosDiagnosticsConfig
{
  // diagnostics toggle
  bool enable_diagnostics{false};

  // corresponds to launch arguments
  bool enable_all_diagnostics{false};
  bool enable_node_time_diagnostics{false};
  bool enable_msg_time_diagnostics{false};
  bool enable_increasing_msg_time_diagnostics{false};

  // enable basic diagnostics for all topics, triggered by an environment variable
  bool enable_all_topic_diagnostics{false};

  // Rate (Hz) at which to publish diagnostics to a ROS topic
  float diagnostics_publish_rate{1.0};

  // Window size of the mean filter in terms of number of messages received
  int filter_window_size{100};

  // Map of topic name and corresponding expected time difference between
  // messages in microseconds
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
  : node_(node), nitros_type_manager_(nitros_type_manager),
    gxf_component_info_(gxf_component_info),
    supported_data_formats_(supported_data_formats), config_(config) {}

  NitrosPublisherSubscriberBase(
    rclcpp::Node & node, const gxf_context_t context,
    std::shared_ptr<NitrosTypeManager> nitros_type_manager,
    const gxf::optimizer::ComponentInfo & gxf_component_info,
    const std::vector<std::string> & supported_data_formats,
    const NitrosPublisherSubscriberConfig & config)
  : node_(node), context_(context),
    nitros_type_manager_(nitros_type_manager),
    gxf_component_info_(gxf_component_info),
    supported_data_formats_(supported_data_formats), config_(config) {}

  const NitrosPublisherSubscriberConfig getConfig() const {return config_;}

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

  // Get the negotiated data format or the compatible format if negotiation
  // failed
  std::string getFinalDataFormat() const
  {
    if (negotiated_data_format_.empty()) {
      return config_.compatible_data_format;
    }
    return negotiated_data_format_;
  }

  // Getter for the GXF context
  gxf_context_t getContext() {return context_;}

  // Setter for the parent GXF context
  void setContext(const gxf_context_t context) {context_ = context;}

  // Setter for the frame_id passthrough map
  void setFrameIdMap(
    std::shared_ptr<std::map<gxf::optimizer::ComponentKey, std::string>>
    frame_id_map_ptr)
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
        "[NitrosPublisherSubscriberBase] Failed to resolve entity "
        "(eid=%ld) (error=%s)",
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
        supported_data_formats_.begin(), supported_data_formats_.end(),
        config_.compatible_data_format) == supported_data_formats_.end())
    {
      std::stringstream error_msg;
      error_msg
        << "[NitrosPublisherSubscriberBase] Specified compatible format "
        << "\"" << config_.compatible_data_format.c_str()
        << "\" was not listed in "
        << "the supported format list: [";
      for (size_t i = 0; i < supported_data_formats_.size(); i++) {
        if (i > 0) {
          error_msg << ", ";
        }
        error_msg << supported_data_formats_[i].c_str();
      }
      error_msg << "]";
      RCLCPP_ERROR(node_.get_logger(), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  // update node (pub/sub) time queue which is used for frame rate calculation
  void updateNodeTimeQueue(const int64_t timestamp_diff_node_us)
  {
    interarrival_time_queue_node_.push(timestamp_diff_node_us);
    sum_interarrival_time_node_ += timestamp_diff_node_us;
    if (static_cast<int>(interarrival_time_queue_node_.size()) >
      diagnostics_config_.filter_window_size)
    {
      sum_interarrival_time_node_ -= interarrival_time_queue_node_.front();
      interarrival_time_queue_node_.pop();
    }
  }

  // Update node (pub/sub) time diagnostics
  bool updateNodeTimeDiagnostics(const int64_t timestamp_diff_node_us)
  {
    bool error_found = false;
    // calculate difference between time between msgs(using node clock)
    // and expected time between msgs
    int abs_jitter_node =
      std::abs(
      timestamp_diff_node_us -
      diagnostics_config_.topic_name_expected_dt_map[topic_name_]);
    if (abs_jitter_node > diagnostics_config_.jitter_tolerance_us) {
      // Increment jitter outlier count
      num_jitter_outliers_node_++;
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosDiagnostics Node Time]"
        " Difference of time between messages(%li) and expected time between"
        " messages(%i) is out of tolerance(%i) by %i for topic %s. Units "
        "are microseconds.",
        timestamp_diff_node_us,
        diagnostics_config_.topic_name_expected_dt_map[topic_name_],
        diagnostics_config_.jitter_tolerance_us,
        abs_jitter_node, topic_name_.c_str());
    }
    // Update max abs jitter
    max_abs_jitter_node_ = std::max(max_abs_jitter_node_, abs_jitter_node);

    jitter_queue_node_.push(abs_jitter_node);
    sum_jitter_node_ += abs_jitter_node;
    if (static_cast<int>(jitter_queue_node_.size()) >
      diagnostics_config_.filter_window_size)
    {
      sum_jitter_node_ -= jitter_queue_node_.front();
      jitter_queue_node_.pop();
    }
    return error_found;
  }

  void updateMsgTimeQueue(const int64_t timestamp_diff_msg_us)
  {
    interarrival_time_queue_msg_.push(timestamp_diff_msg_us);
    sum_interarrival_time_msg_ += timestamp_diff_msg_us;
    if (static_cast<int>(interarrival_time_queue_msg_.size()) >
      diagnostics_config_.filter_window_size)
    {
      sum_interarrival_time_msg_ -= interarrival_time_queue_msg_.front();
      interarrival_time_queue_msg_.pop();
    }
  }

  bool updateMsgTimeDiagnostics(const int64_t timestamp_diff_msg_us)
  {
    bool error_found = false;
    // calculate difference between time between msgs(using msg clock)
    // and expected time between msgs
    int abs_jitter_msg =
      std::abs(
      static_cast<int>(timestamp_diff_msg_us) -
      diagnostics_config_.topic_name_expected_dt_map[topic_name_]);
    if (abs_jitter_msg > diagnostics_config_.jitter_tolerance_us) {
      // Increment jitter outlier count
      num_jitter_outliers_msg_++;
      frames_dropped_since_last_pub_++;
      prev_drop_ts_ = clock_.now();
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosDiagnostics Message Timestamp]"
        " Difference of time between messages(%lu) and expected "
        "time between"
        " messages(%i) is out of tolerance(%i) by %i for topic %s. "
        "Units are microseconds.",
        timestamp_diff_msg_us,
        diagnostics_config_.topic_name_expected_dt_map[topic_name_],
        diagnostics_config_.jitter_tolerance_us,
        abs_jitter_msg, topic_name_.c_str());
    }

    // Update max abs jitter
    max_abs_jitter_msg_ = std::max(max_abs_jitter_msg_, abs_jitter_msg);
    jitter_queue_msg_.push(abs_jitter_msg);

    sum_jitter_msg_ += abs_jitter_msg;
    if (static_cast<int>(jitter_queue_msg_.size()) >
      diagnostics_config_.filter_window_size)
    {
      sum_jitter_msg_ -= jitter_queue_msg_.front();
      jitter_queue_msg_.pop();
    }

    if (prev_drop_ts_ != std::chrono::steady_clock::time_point::min()) {
      auto time_since_drop =
        std::chrono::duration_cast<std::chrono::seconds>(clock_.now() - prev_drop_ts_);
      if (time_since_drop.count() < kDropWarnTimeoutSeconds) {
        error_found = true;
        status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        frames_dropped_since_last_pub_ = 0;
        update_status_message(status_vec_[0], "FRAME DROP DETECTED");
      }
    }
    return error_found;
  }

  bool updateIncreasingMsgTimeDiagnostics(const uint64_t current_timestamp_msg_us)
  {
    bool error_found = false;
    // Check if message timestamp is increasing
    if (current_timestamp_msg_us <= prev_timestamp_msg_us_) {
      // Increment non increasing message count
      num_non_increasing_msg_++;
      prev_noninc_msg_ts_ = clock_.now();
      RCLCPP_WARN(
        node_.get_logger(),
        "[NitrosDiagnostics Message Timestamp Non Increasing]"
        " Message timestamp is not increasing. Current timestamp: "
        "%lu, Previous timestamp: %lu"
        " for topic %s. Units are microseconds.",
        current_timestamp_msg_us, prev_timestamp_msg_us_, topic_name_.c_str());
    }
    if (prev_noninc_msg_ts_ != std::chrono::steady_clock::time_point::min()) {
      auto time_since_noninc =
        std::chrono::duration_cast<std::chrono::seconds>(clock_.now() - prev_noninc_msg_ts_);
      if (time_since_noninc.count() < kDropWarnTimeoutSeconds) {
        error_found = true;
        status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        frames_dropped_since_last_pub_ = 0;
        update_status_message(status_vec_[0], "NONINCREASING TIMESTAMP");
      }
    }
    return error_found;
  }


// Update diagnostics numbers. To be called in nitros Subscriber and Publisher
  void updateDiagnostics(uint64_t msg_timestamp_ns)
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateDiagnostics() and publishDiagnostics()
    const std::lock_guard<std::mutex> lock(nitros_diagnostics_mutex_);
    // NITROS diagnostics checks message intervals both using the node clock
    // and the message timestamp.
    // All variables name _node refers to the node timestamp checks.
    // All variables name _msg refers to the message timestamp checks.
    status_vec_[0].message = "";
    bool error_found = false;

    // Get the current timestamps in microseconds
    uint64_t current_timestamp_msg_us =
      msg_timestamp_ns / kMicrosecondsToNanoseconds;
    uint64_t current_timestamp_node_us = std::chrono::duration_cast<std::chrono::microseconds>(
      clock_.now().time_since_epoch()).count();

    // we can only calculate frame rate after 2 messages have been received
    if (prev_timestamp_node_us_ != std::numeric_limits<uint64_t>::min()) {
      const int64_t timestamp_diff_node_us = current_timestamp_node_us - prev_timestamp_node_us_;
      updateNodeTimeQueue(timestamp_diff_node_us);
      if (diagnostics_config_.enable_node_time_diagnostics) {
        error_found |= updateNodeTimeDiagnostics(timestamp_diff_node_us);
      }
    }

    if (prev_timestamp_msg_us_ != std::numeric_limits<uint64_t>::min()) {
      const int64_t timestamp_diff_msg_us = current_timestamp_msg_us - prev_timestamp_msg_us_;
      updateMsgTimeQueue(timestamp_diff_msg_us);
      // Do the same checks as above, but for message timestamp
      if (diagnostics_config_.enable_msg_time_diagnostics) {
        error_found |= updateMsgTimeDiagnostics(timestamp_diff_msg_us);
      }
      if (diagnostics_config_.enable_increasing_msg_time_diagnostics) {
        error_found |= updateIncreasingMsgTimeDiagnostics(current_timestamp_msg_us);
      }
    }

    prev_timestamp_node_us_ = current_timestamp_node_us;
    prev_timestamp_msg_us_ = current_timestamp_msg_us;

    // calculate key values for diagnsotics status
    if (sum_interarrival_time_node_ != 0) {
      frame_rate_node_ = kSecondsToMicroseconds /
        (static_cast<float>(sum_interarrival_time_node_) /
        interarrival_time_queue_node_.size());
      mean_abs_jitter_node_ = sum_jitter_node_ / jitter_queue_node_.size();
    } else {
      frame_rate_node_ = 0.0;
      mean_abs_jitter_node_ = 0;
    }
    if (sum_interarrival_time_msg_ != 0) {
      frame_rate_msg_ = kSecondsToMicroseconds /
        (static_cast<float>(sum_interarrival_time_msg_) /
        interarrival_time_queue_msg_.size());
      mean_abs_jitter_msg_ = sum_jitter_msg_ / jitter_queue_msg_.size();
    } else {
      frame_rate_msg_ = 0.0;
      mean_abs_jitter_msg_ = 0;
    }
    // frame dropping warnings
    if (!error_found) {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status_vec_[0].message = "OK";
    }
    outdated_msg_ = false;
  }

  // Function to publish diagnostics to a ROS topic
  void publishDiagnostics()
  {
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateDiagnostics() and publishDiagnostics()
    const std::lock_guard<std::mutex> lock(nitros_diagnostics_mutex_);

    std::vector<diagnostic_msgs::msg::KeyValue> values;
    // publish diagnostics stale if message has not been updated since the last call
    if (outdated_msg_) {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status_vec_[0].message = "DIAGNOSTICS STALE";
    } else {
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_non_increasing_msg")
        .value(std::to_string(num_non_increasing_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_msg")
        .value(std::to_string(num_jitter_outliers_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_node")
        .value(std::to_string(num_jitter_outliers_node_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_msg")
        .value(std::to_string(max_abs_jitter_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_node")
        .value(std::to_string(max_abs_jitter_node_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_msg")
        .value(std::to_string(mean_abs_jitter_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_node")
        .value(std::to_string(mean_abs_jitter_node_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_msg")
        .value(std::to_string(frame_rate_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_node")
        .value(std::to_string(frame_rate_node_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("total_dropped_frames")
        .value(std::to_string(num_jitter_outliers_msg_)));
    }
    status_vec_[0].values = values;

    // timestamp from std::chrono
    uint64_t time = std::chrono::duration_cast<std::chrono::nanoseconds>(
      clock_.now() - t_start_)
      .count();
    uint32_t time_seconds =
      time / (kSecondsToMicroseconds * kMicrosecondsToNanoseconds);
    uint32_t time_ns = time - time_seconds;

    diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
    std_msgs::msg::Header header;
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = time_seconds;
    timestamp.nanosec = time_ns;
    header.stamp = timestamp;
    diagnostic_msg.header = header;
    diagnostic_msg.status = status_vec_;

    diagnostic_publisher_->publish(diagnostic_msg);
    outdated_msg_ = true;
  }

  // Initialize diagnostics variables
  void initDiagnostics()
  {
    topic_name_ = config_.topic_name;
    diagnostic_msgs::msg::DiagnosticStatus topic_status;
    topic_status.name = node_.get_name();
    topic_status.name.append(node_.get_namespace());
    topic_status.name.append("/");
    topic_status.name.append(config_.topic_name);
    topic_status.hardware_id = nvidiaID;
    topic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    topic_status.message = "UNDEFINED STATE";
    status_vec_.push_back(topic_status);
    frames_dropped_since_last_pub_ = 0;

    t_start_ = clock_.now();

    // Initialize varibles to min and zero as a flag to detect no messages have
    // been received
    prev_drop_ts_ = std::chrono::steady_clock::time_point::min();
    prev_noninc_msg_ts_ = std::chrono::steady_clock::time_point::min();
    prev_timestamp_node_us_ = std::numeric_limits<uint64_t>::min();
    prev_timestamp_msg_us_ = std::numeric_limits<uint64_t>::min();
    sum_interarrival_time_node_ = 0;
    sum_interarrival_time_msg_ = 0;
    sum_jitter_node_ = 0;
    sum_jitter_msg_ = 0;
    max_abs_jitter_node_ = 0;
    max_abs_jitter_msg_ = 0;
    num_jitter_outliers_node_ = 0;
    num_jitter_outliers_msg_ = 0;
    num_non_increasing_msg_ = 0;
    outdated_msg_ = true;

    diagnostic_publisher_ =
      node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    diagnostics_publisher_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int>(
          1000 / diagnostics_config_.diagnostics_publish_rate)),
      [this]() -> void {publishDiagnostics();});
  }

protected:
  // The parent ROS 2 node
  rclcpp::Node & node_;

  // The parent GXF context
  gxf_context_t context_ = nullptr;

  // Nitros type manager
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;

  // The info of the GXF component that's associated to this Nitros
  // publisher/subscriber
  gxf::optimizer::ComponentInfo gxf_component_info_;

  // Supported data formats
  std::vector<std::string> supported_data_formats_;

  // Configurations for creating the corresponding publisher/subscriber
  NitrosPublisherSubscriberConfig config_;

  // Negotiated data format
  std::string negotiated_data_format_;

  // Frame ID map
  std::shared_ptr<std::map<gxf::optimizer::ComponentKey, std::string>>
  frame_id_map_ptr_;

  // Mutex lock to prevent simultaneous access of common parameters
  // used by updateDiagnostics() and publishDiagnostics()
  std::mutex nitros_diagnostics_mutex_;

  // NITROS diagnostics variables
  // Configurations for a Nitros diagnostics
  NitrosDiagnosticsConfig diagnostics_config_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_publisher_;

  // diagnostics message elements
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec_;

  // Clock object used to retrived current timestamps
  std::chrono::steady_clock clock_;
  std::chrono::steady_clock::time_point t_start_;

  // Queue to store time between messages to implement windowed mean filter
  std::queue<int> interarrival_time_queue_node_;
  std::queue<uint64_t> interarrival_time_queue_msg_;

  // Queue to store message jitter to implement windowed mean filter
  // Jitter is the difference between the time between msgs(dt)
  // calculated from fps specified in NITROS diagnostics ROS param
  // and measured using node clock
  std::queue<int> jitter_queue_node_;
  std::queue<int> jitter_queue_msg_;


  // Sum of the message interarrival times received on this topic within the
  // configured window
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

  // Number of dropped frames without error warnings
  // idk if we need a full 64 bit integer here?
  uint64_t frames_dropped_since_last_pub_;

  // Prev timestamp stored for calculating frame rate
  // Prev node timstamp
  std::chrono::time_point<std::chrono::steady_clock> prev_drop_ts_, prev_noninc_msg_ts_;
  // Prev message timestamp in microseconds
  uint64_t prev_timestamp_msg_us_, prev_timestamp_node_us_;

  // NITROS diagnostics publisher timer
  rclcpp::TimerBase::SharedPtr diagnostics_publisher_timer_;

  // data tracking variables for diagnostics
  std::string topic_name_;
  double frame_rate_node_, frame_rate_msg_;
  int32_t mean_abs_jitter_node_, mean_abs_jitter_msg_;
  bool outdated_msg_;

  void update_status_message(diagnostic_msgs::msg::DiagnosticStatus & status, std::string update)
  {
    if (status.message.empty()) {
      status.message = update;
    } else {
      status.message.append(", ").append(update);
    }
  }
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_PUBLISHER_SUBSCRIBER_BASE_HPP_
