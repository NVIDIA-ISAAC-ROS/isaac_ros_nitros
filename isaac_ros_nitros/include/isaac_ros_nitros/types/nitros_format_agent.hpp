/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_FORMAT_AGENT_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_FORMAT_AGENT_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "negotiated/negotiated_publisher.hpp"
#include "negotiated/negotiated_subscription.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"


constexpr char LOGGER_SUFFIX[] = "NitrosFormatAgent";

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

struct NitrosFormatCallbacks
{
  // Publisher callbacks
  // Create a compatible publisher for T
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<rclcpp::PublisherBase> & compatible_pub,
      const std::string & topic_name,
      const rclcpp::QoS & qos,
      const rclcpp::PublisherOptions & options)>
  createCompatiblePublisherCallback{nullptr};

  // Add a compatible publisher of type T to a negotiated publisher
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
      std::shared_ptr<rclcpp::PublisherBase> compatible_pub,
      const double weight)>
  addCompatiblePublisherCallback{nullptr};

  // Add T to a negotiated publisher as a supported format
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
      const double weight,
      const rclcpp::QoS & qos,
      const rclcpp::PublisherOptions & options)>
  addPublisherSupportedFormatCallback{nullptr};

  // Publish a message of type T via a negotiated publisher
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
      NitrosTypeBase & base_msg)>
  negotiatedPublishCallback{nullptr};

  // Publish a message of type T via a compatible publisher
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<rclcpp::PublisherBase> compatible_pub,
      NitrosTypeBase & base_msg)>
  compatiblePublishCallback{nullptr};


  // Subscriber callbacks
  // Create a compatible subscriber for T
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<rclcpp::SubscriptionBase> & compatible_sub,
      const std::string & topic_name,
      const rclcpp::QoS & qos,
      // std::function<void(std::shared_ptr<DATA_TYPE_NAME::MsgT>)> & subscriber_callback,
      std::function<void(NitrosTypeBase &, const std::string data_format_name)> subscriber_callback,
      const rclcpp::SubscriptionOptions & options)>
  createCompatibleSubscriberCallback{nullptr};

  // Remove a compatible subscriber of type T from a negotiated subscriber
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
      std::shared_ptr<rclcpp::SubscriptionBase> compatible_sub)>
  removeCompatibleSubscriberCallback {nullptr};

  // Add a compatible subscriber of type T to a negotiated subscriber
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
      std::shared_ptr<rclcpp::SubscriptionBase> compatible_sub,
      const double weight)>
  addCompatibleSubscriberCallback{nullptr};

  // Add T to a negotiated subscriber as a supported format
  std::function<
    void(
      rclcpp::Node & node,
      std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
      const double weight,
      const rclcpp::QoS & qos,
      // std::function<void(std::shared_ptr<DATA_TYPE_NAME::MsgT>)> subscriber_callback,
      std::function<void(NitrosTypeBase &, const std::string data_format_name)> subscriber_callback,
      const rclcpp::SubscriptionOptions & options)>
  addSubscriberSupportedFormatCallback{nullptr};

  // Utilities
  // Get T's extension list
  std::function<
    std::vector<std::pair<std::string, std::string>>()>
  getExtensions{nullptr};

  // Get the corresponding ROS type name for the format T
  std::function<std::string()> getROSTypeName{nullptr};
};

template<typename T>
class NitrosFormatAgent
{
public:
  // Everything in this class is static and templated
  NitrosFormatAgent() = delete;

  static NitrosFormatCallbacks GetFormatCallbacks()
  {
    return {
      // createCompatiblePublisherCallback
      std::bind(
        &NitrosFormatAgent<T>::createCompatiblePublisherCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5
      ),

      // addCompatiblePublisherCallback
      std::bind(
        &NitrosFormatAgent<T>::addCompatiblePublisherCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4
      ),

      // addPublisherSupportedFormatCallback
      std::bind(
        &NitrosFormatAgent<T>::addPublisherSupportedFormatCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5
      ),

      // negotiatedPublishCallback
      std::bind(
        &NitrosFormatAgent<T>::negotiatedPublishCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      ),

      // compatiblePublishCallback
      std::bind(
        &NitrosFormatAgent<T>::compatiblePublishCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      ),

      // createCompatibleSubscriberCallback
      std::bind(
        &NitrosFormatAgent<T>::createCompatibleSubscriberCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5,
        std::placeholders::_6
      ),

      // removeCompatibleSubscriberCallback
      std::bind(
        &NitrosFormatAgent<T>::removeCompatibleSubscriberCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
      ),

      // addCompatibleSubscriberCallback
      std::bind(
        &NitrosFormatAgent<T>::addCompatibleSubscriberCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4
      ),

      // addSubscriberSupportedFormatCallback
      std::bind(
        &NitrosFormatAgent<T>::addSubscriberSupportedFormatCallback,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5,
        std::placeholders::_6
      ),

      // getExtensions
      std::bind(
        &NitrosFormatAgent<T>::getExtensions
      ),

      // getROSTypeName
      std::bind(
        &NitrosFormatAgent<T>::getROSTypeName
      ),
    };
  }

  // Publisher callbacks
  // Create a compatible publisher for T
  static void createCompatiblePublisherCallback(
    rclcpp::Node & node,
    std::shared_ptr<rclcpp::PublisherBase> & compatible_pub,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options)
  {
    compatible_pub = node.create_publisher<typename T::MsgT>(
      topic_name,
      qos,
      options);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Created a compatible publisher (topic_name=%s)",
      compatible_pub->get_topic_name());
  }

  // Add a compatible publisher of type T to a negotiated publisher
  static void addCompatiblePublisherCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
    std::shared_ptr<rclcpp::PublisherBase> compatible_pub,
    const double weight)
  {
    auto cast_compatible_pub =
      std::static_pointer_cast<rclcpp::Publisher<typename T::MsgT>>(compatible_pub);
    negotiated_pub->add_compatible_publisher(
      cast_compatible_pub,
      T::supported_type_name,
      weight);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Added a compatible publisher (topic_name=%s) to a negotiated publisher",
      compatible_pub->get_topic_name());
  }

  // Add T to a negotiated publisher as a supported format
  static void addPublisherSupportedFormatCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
    const double weight,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options)
  {
    negotiated_pub->add_supported_type<T>(
      weight,
      qos,
      options);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Added a supported format \"%s\" to a negotiated publisher",
      T::supported_type_name.c_str());
  }

  // Publish a message of type T via a negotiated publisher
  static void negotiatedPublishCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub,
    NitrosTypeBase & base_msg)
  {
    auto msg = static_cast<typename T::MsgT &>(base_msg);
    negotiated_pub->publish<T>(msg);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Published a message via the negotiated publisher");
  }

  // Publish a message of type T via a compatible publisher
  static void compatiblePublishCallback(
    rclcpp::Node & node,
    std::shared_ptr<rclcpp::PublisherBase> compatible_pub,
    NitrosTypeBase & base_msg)
  {
    auto msg = static_cast<typename T::MsgT &>(base_msg);
    auto cast_compatible_pub =
      static_cast<rclcpp::Publisher<typename T::MsgT> *>(compatible_pub.get());
    cast_compatible_pub->publish(msg);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Published a message via the compatible publisher (topic_name=%s)",
      compatible_pub->get_topic_name());
  }

  // Subscriber callbacks
  // Create a compatible subscriber for T
  static void createCompatibleSubscriberCallback(
    rclcpp::Node & node,
    std::shared_ptr<rclcpp::SubscriptionBase> & compatible_sub,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::function<void(NitrosTypeBase &, const std::string data_format_name)> subscriber_callback,
    const rclcpp::SubscriptionOptions & options)
  {
    std::function<void(std::shared_ptr<const typename T::MsgT>)> internal_subscriber_callback =
      std::bind(
      &NitrosFormatAgent<T>::subscriberCallback,
      std::placeholders::_1,
      subscriber_callback);
    compatible_sub = node.create_subscription<typename T::MsgT>(
      topic_name,
      qos,
      internal_subscriber_callback,
      options);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Created a compatible subscriber (topic_name=%s)",
      compatible_sub->get_topic_name());
  }

  // Remove a compatible subscriber of type T from a negotiated subscriber
  static void removeCompatibleSubscriberCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
    std::shared_ptr<rclcpp::SubscriptionBase> compatible_sub)
  {
    auto cast_compatible_sub =
      std::static_pointer_cast<rclcpp::Subscription<typename T::MsgT>>(compatible_sub);
    negotiated_sub->remove_compatible_subscription(
      cast_compatible_sub, T::supported_type_name);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Removed a compatible subscriber (topic_name=%s) from a negotiated subscriber",
      compatible_sub->get_topic_name());
  }

  // Add a compatible subscriber of type T to a negotiated subscriber
  static void addCompatibleSubscriberCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
    std::shared_ptr<rclcpp::SubscriptionBase> compatible_sub,
    const double weight)
  {
    auto cast_compatible_sub =
      std::static_pointer_cast<rclcpp::Subscription<typename T::MsgT>>(compatible_sub);
    negotiated_sub->add_compatible_subscription(
      cast_compatible_sub,
      T::supported_type_name,
      weight);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Added a compatible subscriber (topic_name=%s) to a negotiated subscriber",
      compatible_sub->get_topic_name());
  }

  // Add T to a negotiated subscriber as a supported format
  static void addSubscriberSupportedFormatCallback(
    rclcpp::Node & node,
    std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub,
    const double weight,
    const rclcpp::QoS & qos,
    // std::function<void(std::shared_ptr<DATA_TYPE_NAME::MsgT>)> subscriber_callback,
    std::function<void(NitrosTypeBase &, const std::string data_format_name)> subscriber_callback,
    const rclcpp::SubscriptionOptions & options)
  {
    std::function<void(std::shared_ptr<const typename T::MsgT>)> internal_subscriber_callback =
      std::bind(
      &NitrosFormatAgent<T>::subscriberCallback,
      std::placeholders::_1,
      subscriber_callback);
    negotiated_sub->add_supported_callback<T>(
      weight,
      qos,
      internal_subscriber_callback,
      options);

    RCLCPP_DEBUG(
      node.get_logger().get_child(LOGGER_SUFFIX).get_child(T::supported_type_name),
      "Added a supported format \"%s\" to a negotiated subscriber",
      T::supported_type_name.c_str());
  }

  static std::vector<std::pair<std::string, std::string>> getExtensions()
  {
    return T::MsgT::GetExtensions();
  }

  // Utilities
  // Get the corresponding ROS type name for the format T
  static std::string getROSTypeName()
  {
    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    return rosidl_generator_traits::name<ROSMessageType>();
  }

private:
  static void subscriberCallback(
    const std::shared_ptr<const typename T::MsgT> msg,
    std::function<void(
      NitrosTypeBase &,
      const std::string data_format_name)> subscriber_callback)
  {
    auto base_msg = (NitrosTypeBase)(*msg.get());
    subscriber_callback(
      base_msg,
      T::supported_type_name
    );
  }
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_FORMAT_AGENT_HPP_
