/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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

NitrosSubscriber::NitrosSubscriber(
  rclcpp::Node & node,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config)
: NitrosPublisherSubscriberBase(
    node, nitros_type_manager, gxf_component_info, supported_data_formats, config)
{
  if (config_.type == NitrosPublisherSubscriberType::NOOP) {
    return;
  }

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
    node, nitros_type_manager, gxf_component_info, supported_data_formats, config)
{
  setContext(context);
}

NitrosSubscriber::NitrosSubscriber(
  rclcpp::Node & node,
  const gxf_context_t context,
  std::shared_ptr<NitrosTypeManager> nitros_type_manager,
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config,
  const NitrosStatisticsConfig & statistics_config)
: NitrosSubscriber(
    node, context, nitros_type_manager, gxf_component_info, supported_data_formats, config)
{
  statistics_config_ = statistics_config;

  if (statistics_config_.enable_statistics) {
    // Initialize statistics variables and message fields
    statistics_msg_.is_subscriber = false;
    initStatistics();
  }
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

void NitrosSubscriber::createCompatibleSubscriber()
{
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

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

    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosSubscriber] Use the negotiated data format: \"%s\"",
      negotiated_data_format_.c_str());
  }
}

void NitrosSubscriber::setReceiverPointer(void * gxf_receiver_ptr)
{
  gxf_receiver_ptr_ = reinterpret_cast<nvidia::gxf::Receiver *>(gxf_receiver_ptr);
}

bool NitrosSubscriber::pushEntity(const int64_t eid)
{
  auto msg_entity = nvidia::gxf::Entity::Shared(context_, eid);
  gxf_receiver_ptr_->push(std::move(msg_entity.value()));

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
  #if defined(USE_NVTX)
  std::stringstream nvtx_tag_name;
  nvtx_tag_name <<
    "[" << node_.get_name() << "] NitrosSubscriber::subscriberCallback(" <<
    config_.topic_name << ", t=" <<
    getTimestamp(msg_base) << ")";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_PURPLE);
  #endif

  if (statistics_config_.enable_statistics) {
    updateStatistics();
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

  if (gxf_receiver_ptr_ == nullptr) {
    RCLCPP_INFO(
      node_.get_logger(),
      "[NitrosSubscriber] Received a message but the application receiver's pointer is "
      "not yet set.");
    return;
  }

  if (config_.callback != nullptr) {
    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Calling user-defined callback for an Nitros-typed "
      "message (eid=%ld)", msg_base.handle);
    config_.callback(context_, msg_base);
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

  pushEntity(msg_base.handle);

  #if defined(USE_NVTX)
  nvtxRangePopWrapper();
  #endif
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
