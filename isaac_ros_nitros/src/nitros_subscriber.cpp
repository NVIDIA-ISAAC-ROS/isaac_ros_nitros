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
#include "isaac_ros_nitros/types/types.hpp"

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
  const gxf::optimizer::ComponentInfo & gxf_component_info,
  const std::vector<std::string> & supported_data_formats,
  const NitrosPublisherSubscriberConfig & config)
: NitrosPublisherSubscriberBase(
    node, gxf_component_info, supported_data_formats, config)
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
  #define SUB_HELPER(DATA_TYPE_NAME) \
  if (data_format == DATA_TYPE_NAME::supported_type_name) { \
    if (data_format == config_.compatible_data_format) { \
      auto compatible_sub = \
        std::static_pointer_cast<rclcpp::Subscription<DATA_TYPE_NAME::MsgT>>(compatible_sub_); \
      negotiated_sub_->add_compatible_subscription( \
        compatible_sub, config_.compatible_data_format, \
        weight); \
      RCLCPP_DEBUG( \
        node_.get_logger(), \
        "[NitrosSubscriber] Added a compatible subscriber: " \
        "topic_name=\"%s\", data_format=\"%s\"", \
        compatible_sub_->get_topic_name(), config_.compatible_data_format.c_str()); \
    } else { \
      std::function<void(std::shared_ptr<DATA_TYPE_NAME::MsgT>)> subscriber_callback = \
        std::bind( \
        &NitrosSubscriber::subscriberCallback<DATA_TYPE_NAME::MsgT>, \
        this, \
        std::placeholders::_1, \
        data_format); \
      negotiated_sub_->add_supported_callback<DATA_TYPE_NAME>( \
        weight, \
        config_.qos, \
        subscriber_callback, \
        sub_options); \
      RCLCPP_DEBUG( \
        node_.get_logger(), \
        "[NitrosSubscriber] Added a supported data format: " \
        "topic_name=\"%s\", data_format=\"%s\"", \
        compatible_sub_->get_topic_name(), data_format.c_str()); \
    } \
    return; \
  }
  FOREACH_NITROS_DATA_FORMAT(SUB_HELPER);

  std::stringstream error_msg;
  error_msg <<
    "[NitrosSubscriber] Could not identify the supported data foramt: " <<
    "\"" << data_format.c_str() << "\"";
  RCLCPP_ERROR(
    node_.get_logger(), error_msg.str().c_str());
  throw std::runtime_error(error_msg.str().c_str());
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

  #define COMP_SUB_HELPER(DATA_TYPE_NAME) \
  if (config_.compatible_data_format == DATA_TYPE_NAME::supported_type_name) { \
    std::function<void(std::shared_ptr<DATA_TYPE_NAME::MsgT>)> subscriber_callback = \
      std::bind( \
      &NitrosSubscriber::subscriberCallback<DATA_TYPE_NAME::MsgT>, \
      this, \
      std::placeholders::_1, \
      config_.compatible_data_format); \
    compatible_sub_ = node_.create_subscription<DATA_TYPE_NAME::MsgT>( \
      config_.topic_name, \
      config_.qos, \
      subscriber_callback, \
      sub_options); \
    return; \
  }
  FOREACH_NITROS_DATA_FORMAT(COMP_SUB_HELPER);

  std::stringstream error_msg;
  error_msg <<
    "[NitrosSubscriber] Could not identify the compatible data foramt: " <<
    "\"" << config_.compatible_data_format.c_str() << "\"";
  RCLCPP_ERROR(
    node_.get_logger(), error_msg.str().c_str());
  throw std::runtime_error(error_msg.str().c_str());
}

void NitrosSubscriber::postNegotiationCallback()
{
  if (config_.type != NitrosPublisherSubscriberType::NEGOTIATED) {
    return;
  }

  auto topics_info = negotiated_sub_->get_negotiated_topics_info();
  if (!topics_info.success || topics_info.negotiated_topics.size() == 0) {
    RCLCPP_INFO(node_.get_logger(), "[NitrosSubscriber] Negotiation failed");
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
      #define REMOVE_COMP_SUBSCRIBER_HELPER(DATA_TYPE_NAME) \
  if (config_.compatible_data_format == DATA_TYPE_NAME::supported_type_name) { \
    auto compatible_sub = \
      std::static_pointer_cast<rclcpp::Subscription<DATA_TYPE_NAME::MsgT>>(compatible_sub_); \
    negotiated_sub_->remove_compatible_subscription( \
      compatible_sub, config_.compatible_data_format); \
    RCLCPP_DEBUG( \
      node_.get_logger(), \
      "[NitrosSubscriber] Removed a compatible subscriber: " \
      "topic_name=\"%s\", data_format=\"%s\"", \
      compatible_sub_->get_topic_name(), config_.compatible_data_format.c_str()); \
  }
      FOREACH_NITROS_DATA_FORMAT(REMOVE_COMP_SUBSCRIBER_HELPER);
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

template<typename T>
void NitrosSubscriber::subscriberCallback(
  const std::shared_ptr<T> msg,
  const std::string data_format_name)
{
  #if defined(USE_NVTX)
  std::stringstream nvtx_tag_name;
  nvtx_tag_name << "[" << node_.get_name() << "] NitrosSubscriber::subscriberCallback";
  nvtxRangePushWrapper(nvtx_tag_name.str().c_str(), CLR_PURPLE);
  #endif

  RCLCPP_DEBUG(node_.get_logger(), "[NitrosSubscriber] Received a Nitros-typed messgae");
  RCLCPP_DEBUG(node_.get_logger(), "[NitrosSubscriber] \teid: %ld", msg->handle);
  RCLCPP_DEBUG(
    node_.get_logger(), "[NitrosSubscriber] \tdata_format_name: %s",
    data_format_name.c_str());
  RCLCPP_DEBUG(node_.get_logger(), "[NitrosSubscriber] \tmsg: %s", msg->data_format_name.c_str());
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
      "message (eid=%ld)", msg->handle);
    config_.callback(context_, *msg.get());
  }

  if (frame_id_map_ptr_ != nullptr) {
    std::string frame_id_source_key = config_.frame_id_source_key.empty() ?
      GenerateComponentKey(gxf_component_info_) : config_.frame_id_source_key;
    (*frame_id_map_ptr_.get())[frame_id_source_key] = msg->frame_id;

    RCLCPP_DEBUG(
      node_.get_logger(),
      "[NitrosSubscriber] Updated frame_id=%s",
      (*frame_id_map_ptr_.get())[frame_id_source_key].c_str());
  }

  pushEntity(msg->handle);

  #if defined(USE_NVTX)
  nvtxRangePopWrapper();
  #endif
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
