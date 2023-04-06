/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char PACKAE_NAME[] = "isaac_ros_nitros_disparity_image_type";
constexpr char FORWARD_FORMAT[] = "nitros_disparity_image_bgr8";

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
class NitrosDisparityImageForwardNode : public NitrosNode
{
public:
  explicit NitrosDisparityImageForwardNode(const rclcpp::NodeOptions & options)
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
          {"vault/vault",
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
      PACKAE_NAME)
  {
    std::string compatible_format = declare_parameter<std::string>("compatible_format", "");
    if (!compatible_format.empty()) {
      config_map_["forward/input"].compatible_data_format = compatible_format;
      config_map_["vault/vault"].compatible_data_format = compatible_format;
    }

    registerSupportedType<nvidia::isaac_ros::nitros::NitrosDisparityImage>();

    startNitrosNode();
  }
};
#pragma GCC diagnostic pop

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosDisparityImageForwardNode)
