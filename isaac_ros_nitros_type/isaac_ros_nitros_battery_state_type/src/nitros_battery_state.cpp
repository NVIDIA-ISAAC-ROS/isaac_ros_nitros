// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "extensions/messages/battery_state_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_battery_state_type/nitros_battery_state.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosBatteryState,
  sensor_msgs::msg::BatteryState>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosBatteryState::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_battery_state_parts = nvidia::isaac::GetBatteryStateMessage(
    msg_entity.value());
  if (!maybe_battery_state_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get battery state gxf message"
      " from message entity: " <<
      GxfResultStr(maybe_battery_state_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosBatteryState"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto battery_state_parts = maybe_battery_state_parts.value();
  destination.voltage = battery_state_parts.battery_state->voltage;

  destination.percentage = battery_state_parts.battery_state->charge / 100.0;
  if (battery_state_parts.battery_state->is_charging) {
    destination.power_supply_status = 1;  // POWER_SUPPLY_STATUS_CHARGING = 1
  } else {
    destination.power_supply_status = 3;  // POWER_SUPPLY_STATUS_NOT_CHARGING = 3
  }

  // The following fields are unable to convert:
  // destination.temperature
  // destination.current
  // destination.charge
  // destination.capacity
  // destination.design_capacity
  // destination.power_supply_health
  // destination.power_supply_technology
  // destination.present
  // destination.cell_voltage
  // destination.cell_temperature
  // destination.location
  // destination.serial_number

  // Populate timestamp information back into ROS header
  auto input_timestamp = battery_state_parts.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_ros_message] frame_id = %s", destination.header.frame_id.c_str());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosBatteryState,
  sensor_msgs::msg::BatteryState>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosBatteryState::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // create battery_state gxf msg
  auto maybe_battery_state_parts = nvidia::isaac::CreateBatteryStateMessage(context);
  if (!maybe_battery_state_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to create battery state gxf msg " << GxfResultStr(
      maybe_battery_state_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosBatteryState"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto battery_state_parts = maybe_battery_state_parts.value();

  battery_state_parts.battery_state->charge = source.percentage * 100.0;
  battery_state_parts.battery_state->voltage = source.voltage;
  battery_state_parts.battery_state->health = (source.capacity / source.design_capacity) * 100;
  if (source.power_supply_status == 1) {
    // POWER_SUPPLY_STATUS_CHARGING = 1
    battery_state_parts.battery_state->is_charging = true;
  }
  battery_state_parts.battery_state->reach = 0;

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  battery_state_parts.timestamp->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_custom] frame_id = %s", destination.frame_id.c_str());

  // Set Entity Id
  destination.handle = battery_state_parts.message.eid();
  GxfEntityRefCountInc(context, battery_state_parts.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosBatteryState"),
    "[convert_to_custom] Conversion completed (handle=%ld)", destination.handle);

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
