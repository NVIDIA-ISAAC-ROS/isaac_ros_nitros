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

#include "gxf/std/timestamp.hpp"

#include "isaac_ros_nitros/nitros_publisher.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosPublisherWaitable::NitrosPublisherWaitable(
  rclcpp::Node & node,
  NitrosPublisher & nitros_publisher)
: rclcpp::Waitable(),
  node_(node),
  nitros_publisher_(nitros_publisher)
{}

void NitrosPublisherWaitable::trigger()
{
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosPublisherWaitable::trigger(" <<
    nitros_publisher_.getConfig().topic_name << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_BLUE);

  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  guard_condition_.trigger();

  nvtxRangePopWrapper();
}

bool NitrosPublisherWaitable::is_ready(rcl_wait_set_t * wait_set)
{
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosPublisherWaitable::is_ready(" <<
    nitros_publisher_.getConfig().topic_name << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_BLUE);

  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
    auto rcl_guard_condition = wait_set->guard_conditions[ii];

    if (nullptr == rcl_guard_condition) {
      continue;
    }

    if (&guard_condition_.get_rcl_guard_condition() == rcl_guard_condition) {
      nvtx_tag_name << " = true";
      nvtxMarkExWrapper(nvtx_tag_name.str().c_str(), CLR_BLUE);
      nvtxRangePopWrapper();
      return true;
    }
  }

  nvtx_tag_name << " = false";
  nvtxMarkExWrapper(nvtx_tag_name.str().c_str(), CLR_BLUE);
  nvtxRangePopWrapper();
  return false;
}

void NitrosPublisherWaitable::execute(std::shared_ptr<void> & data)
{
  (void)data;  // unused
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosPublisherWaitable::execute(" <<
    nitros_publisher_.getConfig().topic_name << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_BLUE);

  nitros_publisher_.extractMessagesFromGXF();

  nvtxRangePopWrapper();
}

void NitrosPublisherWaitable::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  guard_condition_.add_to_wait_set(wait_set);
}

NitrosPublisher::NitrosPublisher(
  rclcpp::Node & node,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const negotiated::NegotiatedPublisherOptions & negotiated_pub_options)
: NitrosPublisherSubscriberBase(
    node, nitros_type_manager, gxf_component_info, supported_data_formats, config)
{
  // Bind the callback function to be used by a message relay in GXF graph
  gxf_message_relay_callback_func_ = std::bind(
    &NitrosPublisher::gxfMessageRelayCallback,
    this);

  if (config_.type == NitrosPublisherSubscriberType::NOOP) {
    return;
  }

  // Check and throw error if the specified compatible data format is not supported
  // from the underlying graph
  throwOnUnsupportedCompatibleDataFormat();

  // Create the compatible data format publisher
  createCompatiblePublisher();

  if (config_.type == NitrosPublisherSubscriberType::NON_NEGOTIATED) {
    return;
  }

  // Create a negotiated publisher object
  negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(
    node_,
    compatible_pub_->get_topic_name() + std::string("/nitros"),
    negotiated_pub_options);

  // Add supported data formats (which also adds the compatible publisher)
  double weight = 1.0;
  for (std::string data_format : supported_data_formats_) {
    weight -= 0.1;
    addSupportedDataFormat(
      data_format,
      weight);
  }
}

NitrosPublisher::NitrosPublisher(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const negotiated::NegotiatedPublisherOptions & negotiated_pub_options)
: NitrosPublisher(
    node, nitros_type_manager, gxf_component_info, supported_data_formats, config,
    negotiated_pub_options)
{
  setContext(context);
}

NitrosPublisher::NitrosPublisher(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const negotiated::NegotiatedPublisherOptions & negotiated_pub_options,
  const NitrosStatisticsConfig & statistics_config)
: NitrosPublisher(
    node, context, nitros_type_manager, gxf_component_info, supported_data_formats, config,
    negotiated_pub_options)
{
  statistics_config_ = statistics_config;

  if (statistics_config_.enable_statistics) {
    // Initialize statistics variables and message fields
    statistics_msg_.is_subscriber = false;
    initStatistics();
  }
}

NitrosPublisher::NitrosPublisher(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const NitrosStatisticsConfig & statistics_config)
: NitrosPublisher(
    node, context, nitros_type_manager, {}, supported_data_formats, config,
    {}, statistics_config)
{}

std::shared_ptr<negotiated::NegotiatedPublisher> NitrosPublisher::getNegotiatedPublisher()
{
  return negotiated_pub_;
}

void NitrosPublisher::addSupportedDataFormat(
  const std::string & data_format,
  const double weight)
{
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  if (!nitros_type_manager_->hasFormat(data_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosPublisher] Could not identify the supported data foramt: " <<
      "\"" << data_format.c_str() << "\"";
    RCLCPP_ERROR(
      node_.get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  if (data_format == config_.compatible_data_format) {
    nitros_type_manager_->getFormatCallbacks(data_format).addCompatiblePublisherCallback(
      node_,
      negotiated_pub_,
      compatible_pub_,
      weight
    );

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosPublisher] Added a compatible publisher: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_pub_->get_topic_name(), config_.compatible_data_format.c_str());
  } else {
    nitros_type_manager_->getFormatCallbacks(data_format).addPublisherSupportedFormatCallback(
      node_,
      negotiated_pub_,
      weight,
      config_.qos,
      pub_options);

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosPublisher] Added a supported data format: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_pub_->get_topic_name(), data_format.c_str());
  }
}

void NitrosPublisher::start()
{
  if (config_.type == NitrosPublisherSubscriberType::NEGOTIATED) {
    negotiated_pub_->start();
  }
}

void NitrosPublisher::createCompatiblePublisher()
{
  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  if (!nitros_type_manager_->hasFormat(config_.compatible_data_format)) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosPublisher] Could not identify the compatible data foramt: " <<
      "\"" << config_.compatible_data_format.c_str() << "\"";
    RCLCPP_ERROR(
      node_.get_logger(), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  nitros_type_manager_->getFormatCallbacks(config_.compatible_data_format).
  createCompatiblePublisherCallback(
    node_,
    compatible_pub_,
    config_.topic_name,
    config_.qos,
    pub_options
  );
}

void NitrosPublisher::postNegotiationCallback()
{
  if (config_.type != NitrosPublisherSubscriberType::NEGOTIATED) {
    return;
  }

  auto topics_info = negotiated_pub_->get_negotiated_topics_info();
  if (!topics_info.success || topics_info.negotiated_topics.size() == 0) {
    negotiated_data_format_ = "";
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisher] Negotiation ended with no results");
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisher] Use only the compatible publisher: "
      "topic_name=\"%s\", data_format=\"%s\"",
      compatible_pub_->get_topic_name(), config_.compatible_data_format.c_str());
  } else {
    negotiated_data_format_ = topics_info.negotiated_topics[0].supported_type_name;
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosPublisher] Use the negotiated data format: \"%s\"",
      negotiated_data_format_.c_str());
  }
}

std::function<void(void)> &
NitrosPublisher::getGxfMessageRelayCallbackFunc()
{
  return gxf_message_relay_callback_func_;
}

void NitrosPublisher::setVaultPointer(void * gxf_vault_ptr)
{
  gxf_vault_ptr_ = reinterpret_cast<nvidia::gxf::Vault *>(gxf_vault_ptr);
}

void NitrosPublisher::setMessageRelayPointer(void * gxf_message_relay_ptr)
{
  gxf_message_relay_ptr_ =
    reinterpret_cast<nvidia::isaac_ros::MessageRelay *>(gxf_message_relay_ptr);
}

void NitrosPublisher::startGxfVaultPeriodicPollingTimer()
{
  // Set the Vault data polling timer
  gxf_vault_periodic_polling_timer_ = node_.create_wall_timer(
    std::chrono::microseconds(100),
    [this]() -> void {
      extractMessagesFromGXF();
    });
}

void NitrosPublisher::enableNitrosPublisherWaitable()
{
  auto waitable_interfaces = node_.get_node_waitables_interface();
  waitable_ = std::make_shared<NitrosPublisherWaitable>(
    node_,
    *this
  );
  waitable_interfaces->add_waitable(waitable_, nullptr);
}

void NitrosPublisher::extractMessagesFromGXF()
{
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosPublisher::extractMessagesFromGXF(" <<
    config_.topic_name << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_MAGENTA);

  if (gxf_vault_ptr_ == nullptr && gxf_message_relay_ptr_ == nullptr) {
    nvtxRangePopWrapper();
    return;
  }

  if (gxf_vault_ptr_ != nullptr) {
    // Get messages from Vault
    auto msg_eids = gxf_vault_ptr_->store(100);
    if (msg_eids.size() > 0) {
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosPublisher] Obtained %ld messages from the vault", msg_eids.size());
    }
    for (const auto msg_eid : msg_eids) {
      publish(msg_eid);
      gxf_vault_ptr_->free({msg_eid});
    }
  } else {
    // Get messgaes from MessageRelay
    auto msg_eids = gxf_message_relay_ptr_->store(100);
    if (msg_eids.size() > 0) {
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosPublisher] Obtained %ld messages from the message relay", msg_eids.size());
    }
    for (const auto msg_eid : msg_eids) {
      publish(msg_eid);
      gxf_message_relay_ptr_->free({msg_eid});
    }
  }

  nvtxRangePopWrapper();
}

void NitrosPublisher::gxfMessageRelayCallback()
{
  RCLCPP_DEBUG(node_.get_logger(), "[NitrosPublisher] gxfMessageRelayCallback");
  waitable_->trigger();
}

void NitrosPublisher::publish(const int64_t handle)
{
  std::string frame_id = "";
  if ((frame_id_map_ptr_ != nullptr) && (!config_.frame_id_source_key.empty())) {
    if (frame_id_map_ptr_->count(config_.frame_id_source_key) > 0) {
      frame_id = frame_id_map_ptr_->at(config_.frame_id_source_key);
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[NitrosPublisher] Associating frame_id=\"%s\" to an Nitros-typed "
        "message (eid=%ld)", frame_id.c_str(), handle);
    }
  }

  NitrosTypeBase nitros_msg {
    handle, negotiated_data_format_,
    config_.compatible_data_format, frame_id};

  publish(nitros_msg);
}

void NitrosPublisher::publish(const int64_t handle, const std_msgs::msg::Header & ros_header)
{
  RCLCPP_DEBUG(
    node_.get_logger(),
    "[NitrosPublisher] Publishing an Nitros-typed message with timestamps updated (eid=%ld)",
    handle);

  bool is_timestamp_updated = false;

  auto msg_entity = nvidia::gxf::Entity::Shared(context_, handle);

  uint64_t input_timestamp =
    ros_header.stamp.sec * static_cast<uint64_t>(1e9) +
    ros_header.stamp.nanosec;

  auto maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  if (maybe_input_timestamp) {
    maybe_input_timestamp.value()->acqtime = input_timestamp;
    is_timestamp_updated = true;
  }

  maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (maybe_input_timestamp) {
    maybe_input_timestamp.value()->acqtime = input_timestamp;
    is_timestamp_updated = true;
  }

  if (!is_timestamp_updated) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[NitrosPublisher] Failed to update timestamps in a message entity (eid=%ld) as "
      "no Timestamp component was found",
      handle);
  }

  publish(handle);
}

void NitrosPublisher::publish(
  NitrosTypeBase & base_msg,
  const std_msgs::msg::Header & ros_header)
{
  RCLCPP_DEBUG(
    node_.get_logger(),
    "[NitrosPublisher] Publishing an Nitros-typed message with timestamps updated (eid=%ld)",
    base_msg.handle);

  bool is_timestamp_updated = false;

  auto msg_entity = nvidia::gxf::Entity::Shared(context_, base_msg.handle);

  uint64_t input_timestamp =
    ros_header.stamp.sec * static_cast<uint64_t>(1e9) +
    ros_header.stamp.nanosec;

  auto maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  if (maybe_input_timestamp) {
    maybe_input_timestamp.value()->acqtime = input_timestamp;
    is_timestamp_updated = true;
  }

  maybe_input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (maybe_input_timestamp) {
    maybe_input_timestamp.value()->acqtime = input_timestamp;
    is_timestamp_updated = true;
  }

  if (!is_timestamp_updated) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[NitrosPublisher] Failed to update timestamps in a message entity (eid=%ld) as "
      "no Timestamp component was found",
      base_msg.handle);
  }

  publish(base_msg);
}

void NitrosPublisher::publish(NitrosTypeBase & base_msg)
{
  {
    std::stringstream nvtx_tag_name;
    nvtx_tag_name <<
      "[" << node_.get_name() << "] NitrosPublisher::publish(" <<
      config_.topic_name << ", t=" <<
      getTimestamp(base_msg) << ")";
    nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_PURPLE);
  }

  // Invoke user-defined callback if needed
  if (config_.callback != nullptr) {
    {
      std::stringstream nvtx_tag_name;
      nvtx_tag_name <<
        "[" << node_.get_name() << "] NitrosPublisher::publish(" <<
        config_.topic_name << ")::config_.callback";
      nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_PURPLE);
    }

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosPublisher] Calling user-defined callback for an Nitros-typed "
      "message (eid=%ld)", base_msg.handle);
    config_.callback(context_, base_msg);

    nvtxRangePopWrapper();
  }

  // Skip publishing if its a noop type
  if (config_.type == NitrosPublisherSubscriberType::NOOP) {
    nvtxRangePopWrapper();
    return;
  }

  RCLCPP_DEBUG(
    node_.get_logger(),
    "[NitrosPublisher] Publishing an Nitros-typed message (eid=%ld)", base_msg.handle);

  if (!negotiated_data_format_.empty()) {
    nitros_type_manager_->getFormatCallbacks(negotiated_data_format_).negotiatedPublishCallback(
      node_,
      negotiated_pub_,
      base_msg);
  }

  if (config_.compatible_data_format != negotiated_data_format_) {
    nitros_type_manager_->getFormatCallbacks(config_.compatible_data_format).
    compatiblePublishCallback(
      node_,
      compatible_pub_,
      base_msg);
  }

  if (statistics_config_.enable_statistics) {
    updateStatistics();
  }

  nvtxRangePopWrapper();
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
