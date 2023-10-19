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

#include "isaac_ros_nitros_std_msg_type/nitros_int64.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char PACKAGE_NAME[] = "isaac_ros_nitros_std_msg_type";
constexpr char FORWARD_FORMAT[] = "nitros_int64";

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
class NitrosInt64ForwardNode : public NitrosNode
{
public:
  explicit NitrosInt64ForwardNode(const rclcpp::NodeOptions & options)
  : NitrosNode(
      options,
      // Application graph filename
      "test/config/test_forward_node.yaml",
      // I/O configuration map
        {
          {"forward/input",
            {
              .type = NitrosPublisherSubscriberType::NEGOTIATED,
              .qos = rclcpp::QoS(1),
              .compatible_data_format = FORWARD_FORMAT,
              .topic_name = "topic_forward_input",
              .use_compatible_format_only = true,
            }
          },
          {"sink/sink",
            {
              .type = NitrosPublisherSubscriberType::NEGOTIATED,
              .qos = rclcpp::QoS(1),
              .compatible_data_format = FORWARD_FORMAT,
              .topic_name = "topic_forward_output",
              .use_compatible_format_only = true,
            }
          }
        },
      // Extension specs
      {},
      // Optimizer's rule filenames
      {},
      // Extension so file list
        {
          {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
          {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
          {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
          {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"}
        },
      // Test node package name
      PACKAGE_NAME)
  {
    std::string compatible_format = declare_parameter<std::string>("compatible_format", "");
    if (!compatible_format.empty()) {
      config_map_["forward/input"].compatible_data_format = compatible_format;
      config_map_["sink/sink"].compatible_data_format = compatible_format;
    }

    registerSupportedType<nvidia::isaac_ros::nitros::NitrosInt64>();

    startNitrosNode();
  }
};
#pragma GCC diagnostic pop

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosInt64ForwardNode)
