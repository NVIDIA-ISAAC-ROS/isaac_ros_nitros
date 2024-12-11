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

#include "gxf/core/gxf.h"

#include "isaac_ros_nitros/nitros_subscriber.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char LOGGER_SUFFIX[] = "NitrosSubscriber";
constexpr uint32_t kGxfReceiverPushPollPeriodNs = 100000;  // 100us
constexpr uint32_t kGxfReceiverPushPollWarningPeriodLoopCount = 100000;  // 10s

NitrosSubscriber::NitrosSubscriber(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const NitrosDiagnosticsConfig & diagnostics_config,
  const bool use_callback_group)
: NitrosPublisherSubscriberBase(
    node, context, nitros_type_manager, gxf_component_info, supported_data_formats, config),
  use_callback_group_{use_callback_group}
{
  if (config_.type == NitrosPublisherSubscriberType::NOOP) {
    return;
  }

  diagnostics_config_ = diagnostics_config;
  if (diagnostics_config_.enable_all_topic_diagnostics ||
    (diagnostics_config_.enable_diagnostics &&
    diagnostics_config_.topic_name_expected_dt_map.find(config_.topic_name) !=
    diagnostics_config_.topic_name_expected_dt_map.end()))
  {
    // Initialize diagnostics variables and message fields
    initDiagnostics();
  }

  if (use_callback_group_) {
    callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Created a MutuallyExclusive callback group");
  }

  // Check and throw error if the specified compatible data format is not supported
  // from the underlying graph
  throwOnUnsupportedCompatibleDataFormat();

  // Create the compatible data format subscriber
  createCompatibleSubscriber();

  if (config_.type == NitrosPublisherSubscriberType::NON_NEGOTIATED) {
    return;
  }

  // Create a negotiated subscriber object
  negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
    node_,
    compatible_sub_->get_topic_name() + std::string("/nitros"));

  // Add supported data formats (which also adds the compatible subscriber)
  double weight = 1.0;
  for (std::string data_format : supported_data_formats_) {
    weight -= 0.1;
    addSupportedDataFormat(
      data_format,
      weight);
  }
}

NitrosSubscriber::NitrosSubscriber(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config)
: NitrosSubscriber(
    node, context, nitros_type_manager, gxf_component_info, supported_data_formats, config, {})
{}

NitrosSubscriber::NitrosSubscriber(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const NitrosDiagnosticsConfig & diagnostics_config)
: NitrosSubscriber(
    node, context, nitros_type_manager, {}, supported_data_formats, config, diagnostics_config)
{
  use_gxf_receiver_ = false;
}

std::shared_ptr<negotiated::NegotiatedSubscription> NitrosSubscriber::getNegotiatedSubscriber()
{
  return negotiated_sub_;
}

void NitrosSubscriber::addSupportedDataFormat(
  const std::string & data_format,
  const double weight)
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (use_callback_group_) {
    sub_options.callback_group = callback_group_;
  }

  if (!nitros_type_manager_->hasFormat(data_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosSubscriber] Could not identify the supported data foramt: " <<
      "\"" << data_format.c_str() << "\"";
    RCLCPP_ERROR(
      node_.get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  if (data_format == config_.compatible_data_format) {
    nitros_type_manager_->getFormatCallbacks(data_format).addCompatibleSubscriberCallback(
      node_,
      negotiated_sub_,
      compatible_sub_,
      weight
    );

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Added a compatible subscriber: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_sub_->get_topic_name(), config_.compatible_data_format.c_str());
  } else {
    std::function<void(NitrosTypeBase &, const std::string data_format_name)>
    subscriber_callback =
      std::bind(
      &NitrosSubscriber::subscriberCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2);

    nitros_type_manager_->getFormatCallbacks(data_format).addSubscriberSupportedFormatCallback(
      node_,
      negotiated_sub_,
      weight,
      config_.qos,
      subscriber_callback,
      sub_options);

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Added a supported data format: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_sub_->get_topic_name(), data_format.c_str());
  }
}

void NitrosSubscriber::start()
{
  if (config_.type == NitrosPublisherSubscriberType::NEGOTIATED) {
    negotiated_sub_->start();
  }
}

void NitrosSubscriber::setIsGxfRunning(const bool is_gxf_running)
{
  is_gxf_running_ = is_gxf_running;
}

void NitrosSubscriber::createCompatibleSubscriber()
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (use_callback_group_) {
    sub_options.callback_group = callback_group_;
  }

  if (!nitros_type_manager_->hasFormat(config_.compatible_data_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosSubscriber] Could not identify the compatible data foramt: " <<
      "\"" << config_.compatible_data_format.c_str() << "\"";
    RCLCPP_ERROR(
      node_.get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  std::function<void(NitrosTypeBase &, const std::string)>
  subscriber_callback =
    std::bind(
    &NitrosSubscriber::subscriberCallback,
    this,
    std::placeholders::_1,
    std::placeholders::_2);

  nitros_type_manager_->getFormatCallbacks(config_.compatible_data_format).
  createCompatibleSubscriberCallback(
    node_,
    compatible_sub_,
    config_.topic_name,
    config_.qos,
    subscriber_callback,
    sub_options
  );
}

void NitrosSubscriber::postNegotiationCallback()
{
  if (config_.type != NitrosPublisherSubscriberType::NEGOTIATED) {
    return;
  }

  auto topics_info = negotiated_sub_->get_negotiated_topics_info();
  if (!topics_info.success || topics_info.negotiated_topics.size() == 0) {
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosSubscriber] Negotiation ended with no results");
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosSubscriber] Use the compatible subscriber: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_sub_->get_topic_name(), config_.compatible_data_format.c_str());
    negotiated_data_format_ = "";
  } else {
    negotiated_data_format_ = topics_info.negotiated_topics[0].supported_type_name;

    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosSubscriber] Use the negotiated data format: \"%s\"",
      negotiated_data_format_.c_str());

    if (negotiated_data_format_ != config_.compatible_data_format) {
      // Delete the compatible subscriber as we don't use it anymore
      nitros_type_manager_->getFormatCallbacks(config_.compatible_data_format).
      removeCompatibleSubscriberCallback(
        node_,
        negotiated_sub_,
        compatible_sub_);

      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosSubscriber] Removed a compatible subscriber: "
        "topic_name=\"%s\", data_format=\"%s\"",
        compatible_sub_->get_topic_name(), config_.compatible_data_format.c_str());

      compatible_sub_ = nullptr;
    }
  }
}

void NitrosSubscriber::setReceiverPointer(void * gxf_receiver_ptr)
{
  gxf_receiver_ptr_ = reinterpret_cast<nvidia::gxf::Receiver *>(gxf_receiver_ptr);
}

void NitrosSubscriber::setReceiverPolicy(const size_t policy)
{
  if (gxf_receiver_ptr_ == nullptr) {
    RCLCPP_ERROR(
      node_.get_logger().get_child(LOGGER_SUFFIX),
      "The underlying receiver pointer (\"%s\") is not set when setting its policy.",
      GenerateComponentKey(gxf_component_info_).c_str());
    return;
  }

  gxf_result_t code;
  code = GxfParameterSetUInt64(context_, gxf_receiver_ptr_->cid(), "policy", policy);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      node_.get_logger().get_child(LOGGER_SUFFIX),
      "Failed to set policy for the underlying receiver (\"%s\"): %s",
      GenerateComponentKey(gxf_component_info_).c_str(),
      GxfResultStr(code));
    return;
  }

  RCLCPP_DEBUG(
    node_.get_logger().get_child(LOGGER_SUFFIX),
    "Set policy to %zu for the underlying receiver (\"%s\")",
    policy, GenerateComponentKey(gxf_component_info_).c_str());
}

// Set the capacity for the underlying GXF receiver
void NitrosSubscriber::setReceiverCapacity(const size_t capacity)
{
  if (gxf_receiver_ptr_ == nullptr) {
    RCLCPP_ERROR(
      node_.get_logger().get_child(LOGGER_SUFFIX),
      "The underlying receiver pointer (\"%s\") is not set when setting its capacity.",
      GenerateComponentKey(gxf_component_info_).c_str());
    return;
  }

  gxf_result_t code;
  code = GxfParameterSetUInt64(context_, gxf_receiver_ptr_->cid(), "capacity", capacity);
  if (code != GXF_SUCCESS) {
    RCLCPP_ERROR(
      node_.get_logger().get_child(LOGGER_SUFFIX),
      "Failed to set capacity for the underlying receiver (\"%s\"): %s",
      GenerateComponentKey(gxf_component_info_).c_str(),
      GxfResultStr(code));
    return;
  }

  RCLCPP_DEBUG(
    node_.get_logger().get_child(LOGGER_SUFFIX),
    "Set capacity to %zu for the underlying receiver (\"%s\")",
    capacity, GenerateComponentKey(gxf_component_info_).c_str());
}

bool NitrosSubscriber::pushEntity(const int64_t eid, bool should_block)
{
  gxf_entity_status_t entity_status;
  gxf_result_t code;
  gxf_uid_t gxf_receiver_eid = gxf_receiver_ptr_->eid();
  int loop_count = 0;
  if (should_block) {
    while ((gxf_receiver_ptr_->back_size() + 1) >
      (gxf_receiver_ptr_->capacity() - gxf_receiver_ptr_->size()))
    {
      if (loop_count > 0 && loop_count % kGxfReceiverPushPollWarningPeriodLoopCount == 0) {
        RCLCPP_WARN(
          node_.get_logger().get_child(LOGGER_SUFFIX),
          "%.1fs passed while waiting to push a message entity (eid=%ld) "
          "to the receiver %s",
          (loop_count * (kGxfReceiverPushPollPeriodNs / 1000000000.0)),
          eid, gxf_receiver_ptr_->name());
      }
      rclcpp::sleep_for(std::chrono::nanoseconds(kGxfReceiverPushPollPeriodNs));
      loop_count++;
      // Check the status of the receiver entity in case the graph is terminated.
      code = GxfEntityGetStatus(context_, gxf_receiver_eid, &entity_status);
      if (code != GXF_SUCCESS) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "[NitrosSubscriber] Failed to get the receiver entity (eid=%ld) status: %s",
          gxf_receiver_eid, GxfResultStr(code));
        return false;
      }
    }
  }

  auto msg_entity = nvidia::gxf::Entity::Shared(context_, eid);
  gxf_receiver_ptr_->push(std::move(msg_entity.value()));
  GxfEntityNotifyEventType(context_, gxf_receiver_ptr_->eid(), GXF_EVENT_MESSAGE_SYNC);

  RCLCPP_DEBUG(
    node_.get_logger(),
    "[NitrosSubscriber] Pushed a message entity (eid=%ld) to "
    "the appliation", eid);

  return true;
}

void NitrosSubscriber::subscriberCallback(
  NitrosTypeBase & msg_base,
  const std::string data_format_name)
{
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosSubscriber::subscriberCallback(" <<
    config_.topic_name << ", t=" <<
    getTimestamp(msg_base) << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_PURPLE);

  // Only enable diagnostics if the ROS parameter flag is enabled and
  // the topic has been specified in the topics_list ROS parameter
  if (diagnostics_config_.enable_all_topic_diagnostics ||
    (diagnostics_config_.enable_diagnostics &&
    diagnostics_config_.topic_name_expected_dt_map.find(config_.topic_name) !=
    diagnostics_config_.topic_name_expected_dt_map.end()))
  {
    updateDiagnostics(getTimestamp(msg_base));
  }

  RCLCPP_DEBUG(node_.get_logger(), "[NitrosSubscriber] Received a Nitros-typed messgae");
  RCLCPP_DEBUG(node_.get_logger(), "[NitrosSubscriber] \teid: %ld", msg_base.handle);
  RCLCPP_DEBUG(
    node_.get_logger(), "[NitrosSubscriber] \tdata_format_name: %s",
    data_format_name.c_str());
  RCLCPP_DEBUG(
    node_.get_logger(), "[NitrosSubscriber] \tmsg_base: %s",
    msg_base.data_format_name.c_str());
  RCLCPP_DEBUG(
    node_.get_logger(), "[NitrosSubscriber] \tReceiver's pointer: %p",
    (void *)gxf_receiver_ptr_);

  if (use_gxf_receiver_ && (!is_gxf_running_ || gxf_receiver_ptr_ == nullptr)) {
    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Received a message but the underlying GXF graph is"
      "not yet ready.");
    nvtxRangePopWrapper();
    return;
  }

  if (frame_id_map_ptr_ != nullptr) {
    std::string frame_id_source_key = config_.frame_id_source_key.empty() ?
      GenerateComponentKey(gxf_component_info_) : config_.frame_id_source_key;
    (*frame_id_map_ptr_.get())[frame_id_source_key] = msg_base.frame_id;

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Updated frame_id=%s",
      (*frame_id_map_ptr_.get())[frame_id_source_key].c_str());
  }

  if (config_.callback != nullptr) {
    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Calling user-defined callback for an Nitros-typed "
      "message (eid=%ld)", msg_base.handle);
    config_.callback(context_, msg_base);
  }

  if (use_gxf_receiver_ && gxf_receiver_ptr_ != nullptr && is_gxf_running_) {
    // Push the message to the associated gxf receiver if existed
    pushEntity(msg_base.handle, false);
  }

  nvtxRangePopWrapper();
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
