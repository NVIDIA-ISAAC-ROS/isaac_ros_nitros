// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_MESSAGE_FILTERS_SUBSCRIBER_HPP_
#define ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_MESSAGE_FILTERS_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"

#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros/types/nitros_type_message_filter_traits.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
namespace message_filters
{

template<class NitrosTypeViewT, class NodeType = rclcpp::Node>
class Subscriber
  : public ::message_filters::SubscriberBase<NodeType>,
  public ::message_filters::SimpleFilter<typename NitrosTypeViewT::BaseType>
{
public:
  typedef std::shared_ptr<NodeType> NodePtr;
  typedef typename NitrosTypeViewT::BaseType MessageType;
  typedef ::message_filters::MessageEvent<MessageType const> EventType;

  /**
   * \brief Constructor
   *
   * See the rclcpp::Node::subscribe() variants for more information on the parameters
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  Subscriber(
    NodePtr node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  Subscriber(
    NodeType * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  /**
   * \brief Constructor
   *
   * See the rclcpp::Node::subscribe() variants for more information on the parameters
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  Subscriber(
    NodePtr node,
    const std::string & topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options)
  {
    subscribe(node.get(), topic, qos, options);
  }

  Subscriber(
    NodeType * node,
    const std::string & topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options)
  {
    subscribe(node, topic, qos, options);
  }

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  Subscriber() = default;

  ~Subscriber()
  {
    unsubscribe();
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  void subscribe(
    NodePtr node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) override
  {
    subscribe(node.get(), topic, qos, rclcpp::SubscriptionOptions());
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  // TODO(wjwwood): deprecate in favor of API's that use `rclcpp::QoS` instead.
  void subscribe(
    NodeType * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default) override
  {
    subscribe(node, topic, qos, rclcpp::SubscriptionOptions());
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node::SharedPtr to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  void subscribe(
    NodePtr node,
    const std::string & topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options) override
  {
    subscribe(node.get(), topic, qos, options);
    node_raw_ = nullptr;
    node_shared_ = node;
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The rclcpp::Node to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe
   * \param options The subscription options to use to subscribe.
   */
  // TODO(wjwwood): deprecate in favor of API's that use `rclcpp::QoS` instead.
  void subscribe(
    NodeType * node,
    const std::string & topic,
    const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions options,
    const std::string & compatible_data_format = "",
    const NitrosDiagnosticsConfig & diagnostics_config = {})
  {
    unsubscribe();

    if (!topic.empty()) {
      topic_ = topic;
      rclcpp::QoS rclcpp_qos(rclcpp::QoSInitialization::from_rmw(qos));
      rclcpp_qos.get_rmw_qos_profile() = qos;
      qos_ = qos;
      options_ = options;

      sub_ = std::make_shared<ManagedNitrosSubscriber<NitrosTypeViewT>>(
        node,
        topic,
        compatible_data_format !=
        "" ? compatible_data_format : MessageType::GetDefaultCompatibleFormat(),
        [this](const NitrosTypeViewT & nitrosViewMsg) {
          this->cb(EventType(std::make_shared<const MessageType>(nitrosViewMsg.GetMessage())));
        },
        diagnostics_config,
        rclcpp_qos
      );

      node_raw_ = node;
    }
  }

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
   */
  void subscribe() override
  {
    if (!topic_.empty()) {
      if (node_raw_ != nullptr) {
        subscribe(node_raw_, topic_, qos_, options_);
      } else if (node_shared_ != nullptr) {
        subscribe(node_shared_, topic_, qos_, options_);
      }
    }
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  void unsubscribe() override
  {
    sub_.reset();
  }

  std::string getTopic() const
  {
    return this->topic_;
  }

  /**
   * \brief Returns the internal shared pointer to the ManagedNitrosSubscriber<NitrosTypeViewT> object
   */
  const std::shared_ptr<ManagedNitrosSubscriber<NitrosTypeViewT>> getSubscriber() const
  {
    return sub_;
  }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  template<typename F>
  void connectInput(F & f)
  {
    (void)f;
  }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  void add(const EventType & e)
  {
    (void)e;
  }

private:
  void cb(const EventType & e)
  {
    this->signalMessage(e);
  }

  std::shared_ptr<ManagedNitrosSubscriber<NitrosTypeViewT>> sub_;

  NodePtr node_shared_;
  NodeType * node_raw_ {nullptr};

  std::string topic_;
  rmw_qos_profile_t qos_;
  rclcpp::SubscriptionOptions options_;
};

}  // namespace message_filters
}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_MESSAGE_FILTERS_SUBSCRIBER_HPP_
