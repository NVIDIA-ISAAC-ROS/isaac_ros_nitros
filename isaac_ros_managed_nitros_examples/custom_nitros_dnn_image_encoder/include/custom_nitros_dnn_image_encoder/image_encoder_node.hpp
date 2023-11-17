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

#ifndef CUSTOM_NITROS_DNN_IMAGE_ENCODER__IMAGE_ENCODER_NODE_HPP_
#define CUSTOM_NITROS_DNN_IMAGE_ENCODER__IMAGE_ENCODER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <nvcv/Tensor.hpp>

#include <cvcuda/OpResize.hpp>
#include <cvcuda/OpCvtColor.hpp>
#include <cvcuda/OpConvertTo.hpp>
#include <cvcuda/OpNormalize.hpp>
#include <cvcuda/OpReformat.hpp>

namespace custom_nitros_dnn_image_encoder
{

struct NVCVImageFormat
{
  nvcv::ImageFormat interleaved_format;
  nvcv::ImageFormat planar_float_format;
  nvcv::ImageFormat interleaved_float_format;
};

class ImageEncoderNode : public rclcpp::Node
{
public:
  explicit ImageEncoderNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~ImageEncoderNode();

private:
  void InputCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Subscription to input image messages
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // Publisher for output NitrosTensorList messages
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosTensorList>> nitros_pub_;

  // Name of tensor in NitrosTensorList
  std::string tensor_name_{};
  std::string input_image_encoding_{};
  std::string output_image_encoding_{};
  const uint16_t output_image_width_{};
  const uint16_t output_image_height_{};
  const uint16_t input_image_width_{};
  const uint16_t input_image_height_{};
  std::vector<double> image_mean_{};
  std::vector<double> image_stddev_{};

  const uint16_t batch_size_ = 1;
  uint16_t input_image_channels_ = 0;
  uint16_t output_image_channels_ = 0;

  nvcv::TensorDataStridedCuda::Buffer input_image_buffer_;
  nvcv::Tensor input_image_tensor_;

  nvcv::TensorDataStridedCuda::Buffer output_image_buffer_;
  nvcv::Tensor output_image_tensor_;

  cvcuda::Resize resize_op_;
  cvcuda::CvtColor cvtColor_op_;
  cvcuda::Reformat reformat_op_;
  cvcuda::Normalize norm_op_;
  cvcuda::ConvertTo convert_op_;

  nvcv::Tensor std_dev_tensor_;
  nvcv::Tensor mean_tensor_;
  nvcv::Tensor resized_tensor_;
  nvcv::Tensor color_converted_tensor_;
  nvcv::Tensor float_tensor_;
  nvcv::Tensor norm_tensor_;

  NVCVColorConversionCode color_conversion_code_;
};

}  // namespace custom_nitros_dnn_image_encoder

#endif  // CUSTOM_NITROS_DNN_IMAGE_ENCODER__IMAGE_ENCODER_NODE_HPP_
