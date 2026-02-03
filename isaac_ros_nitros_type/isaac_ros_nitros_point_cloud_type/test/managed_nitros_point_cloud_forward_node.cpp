// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cuda_runtime.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_view.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_builder.hpp"
#include "isaac_ros_common/cuda_stream.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class ManagedNitrosPointCloudForwardNode : public rclcpp::Node
{
public:
  explicit ManagedNitrosPointCloudForwardNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : rclcpp::Node("managed_nitros_point_cloud_forward_node", options),
    nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
          nvidia::isaac_ros::nitros::NitrosPointCloudView>>(
        this,
        "topic_forward_input",
        nvidia::isaac_ros::nitros::nitros_point_cloud_t::supported_type_name,
        std::bind(&ManagedNitrosPointCloudForwardNode::InputCallback, this,
        std::placeholders::_1))},
    nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
          nvidia::isaac_ros::nitros::NitrosPointCloud>>(
        this,
        "topic_forward_output",
        nvidia::isaac_ros::nitros::nitros_point_cloud_t::supported_type_name)}
  {
    CHECK_CUDA_ERROR(
      ::nvidia::isaac_ros::common::initNamedCudaStream(
        cuda_stream_, "managed_nitros_point_cloud_forward_node"),
      "Error initializing CUDA stream");
  }

  ~ManagedNitrosPointCloudForwardNode() = default;

private:
  void InputCallback(const nvidia::isaac_ros::nitros::NitrosPointCloudView & view)
  {
    const size_t point_count = view.GetPointCount();
    const bool has_color = view.HasColor();
    const float * gpu_data_ptr = view.GetPointsData();

    // Calculate buffer size
    const size_t point_size = has_color ? 4 : 3;  // xyzrgb or xyz
    const size_t buffer_size = point_count * point_size * sizeof(float);

    // Allocate output buffer
    float * output_buffer = nullptr;
    CHECK_CUDA_ERROR(
      cudaMallocAsync(&output_buffer, buffer_size, cuda_stream_),
      "Error allocating CUDA buffer for point cloud");

    // Copy data
    CHECK_CUDA_ERROR(
      cudaMemcpyAsync(
        output_buffer, gpu_data_ptr, buffer_size,
        cudaMemcpyDeviceToDevice, cuda_stream_),
      "Error copying point cloud data to CUDA buffer");

    // Wait for copy to complete
    CHECK_CUDA_ERROR(
      cudaStreamSynchronize(cuda_stream_),
      "Error synchronizing CUDA stream");

    std_msgs::msg::Header header;
    header.frame_id = view.GetFrameId();
    header.stamp.sec = view.GetTimestampSeconds();
    header.stamp.nanosec = view.GetTimestampNanoseconds();

    nvidia::isaac_ros::nitros::NitrosPointCloudBuilder builder;
    builder.WithHeader(header);
    builder.WithPoints(output_buffer, point_count, has_color);
    builder.WithReleaseCallback([output_buffer, stream = cuda_stream_]() {
        cudaFreeAsync(output_buffer, stream);
    });
    nvidia::isaac_ros::nitros::NitrosPointCloud nitros_pc = builder.Build();

    nitros_pub_->publish(nitros_pc);
  }

  // Subscription to input NitrosPointCloud messages
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
      nvidia::isaac_ros::nitros::NitrosPointCloudView>> nitros_sub_;

  // Publisher for output NitrosPointCloud messages
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosPointCloud>> nitros_pub_;

  cudaStream_t cuda_stream_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  nvidia::isaac_ros::nitros::ManagedNitrosPointCloudForwardNode)
