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

#include "custom_nitros_dnn_image_encoder/image_encoder_node.hpp"

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"

namespace custom_nitros_dnn_image_encoder
{
std::unordered_map<std::string, NVCVImageFormat> g_str_to_nvcv_image_format({
    {"rgb8", NVCVImageFormat{nvcv::FMT_RGB8, nvcv::FMT_RGBf32p, nvcv::FMT_RGBf32}},
    {"bgr8", NVCVImageFormat{nvcv::FMT_BGR8, nvcv::FMT_BGRf32p, nvcv::FMT_BGRf32}},
    {"rgba8", NVCVImageFormat{nvcv::FMT_RGBA8, nvcv::FMT_RGBAf32p, nvcv::FMT_RGBAf32}},
    {"bgra8", NVCVImageFormat{nvcv::FMT_BGRA8, nvcv::FMT_BGRAf32p, nvcv::FMT_BGRAf32}},
    {"mono8", NVCVImageFormat{nvcv::FMT_Y8, nvcv::FMT_F32, nvcv::FMT_F32}}});

std::map<std::pair<nvcv::ImageFormat, nvcv::ImageFormat>, NVCVColorConversionCode>
get_color_conversion_code({
    {std::make_pair(nvcv::FMT_RGB8, nvcv::FMT_BGR8), NVCV_COLOR_RGB2BGR},
    {std::make_pair(nvcv::FMT_RGB8, nvcv::FMT_RGBA8), NVCV_COLOR_RGB2RGBA},
    {std::make_pair(nvcv::FMT_RGB8, nvcv::FMT_BGRA8), NVCV_COLOR_RGB2BGRA},
    {std::make_pair(nvcv::FMT_RGB8, nvcv::FMT_Y8), NVCV_COLOR_RGB2GRAY},
    {std::make_pair(nvcv::FMT_BGR8, nvcv::FMT_RGB8), NVCV_COLOR_BGR2RGB},
    {std::make_pair(nvcv::FMT_BGR8, nvcv::FMT_RGBA8), NVCV_COLOR_BGR2RGBA},
    {std::make_pair(nvcv::FMT_BGR8, nvcv::FMT_BGRA8), NVCV_COLOR_BGR2BGRA},
    {std::make_pair(nvcv::FMT_BGR8, nvcv::FMT_Y8), NVCV_COLOR_BGR2GRAY},
    {std::make_pair(nvcv::FMT_RGBA8, nvcv::FMT_BGR8), NVCV_COLOR_RGBA2BGR},
    {std::make_pair(nvcv::FMT_RGBA8, nvcv::FMT_RGB8), NVCV_COLOR_RGBA2RGB},
    {std::make_pair(nvcv::FMT_RGBA8, nvcv::FMT_BGRA8), NVCV_COLOR_RGBA2BGRA},
    {std::make_pair(nvcv::FMT_RGBA8, nvcv::FMT_Y8), NVCV_COLOR_RGBA2GRAY},
    {std::make_pair(nvcv::FMT_BGRA8, nvcv::FMT_RGB8), NVCV_COLOR_BGRA2RGB},
    {std::make_pair(nvcv::FMT_BGRA8, nvcv::FMT_RGBA8), NVCV_COLOR_BGRA2RGBA},
    {std::make_pair(nvcv::FMT_BGRA8, nvcv::FMT_BGR8), NVCV_COLOR_BGRA2BGR},
    {std::make_pair(nvcv::FMT_BGRA8, nvcv::FMT_Y8), NVCV_COLOR_BGRA2GRAY},
    {std::make_pair(nvcv::FMT_Y8, nvcv::FMT_RGB8), NVCV_COLOR_GRAY2RGB},
    {std::make_pair(nvcv::FMT_Y8, nvcv::FMT_RGBA8), NVCV_COLOR_GRAY2RGBA},
    {std::make_pair(nvcv::FMT_Y8, nvcv::FMT_BGR8), NVCV_COLOR_GRAY2BGR},
    {std::make_pair(nvcv::FMT_Y8, nvcv::FMT_BGRA8), NVCV_COLOR_GRAY2BGRA},
  });

ImageEncoderNode::ImageEncoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_encoder_node", options),
  sub_{create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&ImageEncoderNode::InputCallback, this,
      std::placeholders::_1))},

  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosTensorList>>(
      this,
      "encoded_tensor",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name)},

  tensor_name_{declare_parameter<std::string>("tensor_name", "input_tensor")},
  input_image_encoding_{declare_parameter<std::string>("input_image_encoding", "bgr8")},
  output_image_encoding_{declare_parameter<std::string>("output_image_encoding", "rgb8")},
  output_image_width_{declare_parameter<uint16_t>("output_image_width", 640)},
  output_image_height_{declare_parameter<uint16_t>("output_image_height", 480)},
  input_image_width_{declare_parameter<uint16_t>("input_image_width", 1920)},
  input_image_height_{declare_parameter<uint16_t>("input_image_height", 1080)},
  image_mean_(declare_parameter<std::vector<double>>("image_mean", {0.0, 0.0, 0.0})),
  image_stddev_(declare_parameter<std::vector<double>>("image_stddev", {1.0, 1.0, 1.0}))
{
  auto iterator = g_str_to_nvcv_image_format.find(input_image_encoding_);
  if (iterator == g_str_to_nvcv_image_format.end()) {
    throw std::runtime_error(
            "Error: received unsupported input image encoding: " +
            input_image_encoding_);
  }
  iterator = g_str_to_nvcv_image_format.find(output_image_encoding_);
  if (iterator == g_str_to_nvcv_image_format.end()) {
    throw std::runtime_error(
            "Error: received unsupported convert to encoding " +
            output_image_encoding_);
  }

  NVCVImageFormat input_image_format = g_str_to_nvcv_image_format[input_image_encoding_];
  NVCVImageFormat output_image_format = g_str_to_nvcv_image_format[output_image_encoding_];

  input_image_channels_ = input_image_format.interleaved_format.numChannels();
  output_image_channels_ = output_image_format.planar_float_format.numChannels();

  if (input_image_format.interleaved_format != output_image_format.interleaved_format) {
    color_conversion_code_ = get_color_conversion_code[std::make_pair(
          input_image_format.interleaved_format, output_image_format.interleaved_format
        )];
  }
  // Initialize input image tensor. The format of input image is expected to be NHWC.
  // Please note, the assignment of the values at each stride index depends upon the format
  // of the image.
  nvcv::Tensor::Requirements reqs = nvcv::Tensor::CalcRequirements(
    batch_size_,
    {input_image_width_, input_image_height_}, input_image_format.interleaved_format);

  input_image_buffer_.strides[3] = sizeof(uint8_t);
  input_image_buffer_.strides[2] = input_image_channels_ * input_image_buffer_.strides[3];
  input_image_buffer_.strides[1] = input_image_width_ * input_image_buffer_.strides[2];
  input_image_buffer_.strides[0] = input_image_height_ * input_image_buffer_.strides[1];
  cudaMalloc(&input_image_buffer_.basePtr, batch_size_ * input_image_buffer_.strides[0]);

  nvcv::TensorDataStridedCuda in_data(nvcv::TensorShape{reqs.shape, reqs.rank, reqs.layout},
    nvcv::DataType{reqs.dtype}, input_image_buffer_);
  input_image_tensor_ = nvcv::TensorWrapData(in_data);

  // Initialize output image tensor. The format of output is expected to be NCHW.
  reqs = nvcv::Tensor::CalcRequirements(
    batch_size_,
    {output_image_width_, output_image_height_}, output_image_format.planar_float_format);

  output_image_buffer_.strides[3] = sizeof(float);
  output_image_buffer_.strides[2] = output_image_width_ * output_image_buffer_.strides[3];
  output_image_buffer_.strides[1] = output_image_height_ * output_image_buffer_.strides[2];
  output_image_buffer_.strides[0] = output_image_channels_ * output_image_buffer_.strides[1];
  cudaMalloc(&output_image_buffer_.basePtr, batch_size_ * output_image_buffer_.strides[0]);

  nvcv::TensorDataStridedCuda out_data(nvcv::TensorShape{reqs.shape, reqs.rank, reqs.layout},
    nvcv::DataType{reqs.dtype}, output_image_buffer_);
  output_image_tensor_ = nvcv::TensorWrapData(out_data);

  mean_tensor_ = nvcv::Tensor(1, {1, 1}, output_image_format.interleaved_float_format);
  std_dev_tensor_ = nvcv::Tensor(1, {1, 1}, output_image_format.interleaved_float_format);
  auto mean_data = mean_tensor_.exportData<nvcv::TensorDataStridedCuda>();
  auto std_dev_data = std_dev_tensor_.exportData<nvcv::TensorDataStridedCuda>();

  std::vector<float> image_mean_float(image_mean_.begin(), image_mean_.end());
  std::vector<float> image_stddev_float(image_stddev_.begin(), image_stddev_.end());

  cudaMemcpy(
    mean_data->basePtr(), image_mean_float.data(),
    3 * sizeof(float), cudaMemcpyHostToDevice
  );
  cudaMemcpy(
    std_dev_data->basePtr(), image_stddev_float.data(),
    3 * sizeof(float), cudaMemcpyHostToDevice
  );

  resized_tensor_ = nvcv::Tensor(
    batch_size_, {output_image_width_, output_image_height_},
    input_image_format.interleaved_format);

  color_converted_tensor_ = nvcv::Tensor(
    batch_size_, {output_image_width_, output_image_height_},
    output_image_format.interleaved_format);

  float_tensor_ = nvcv::Tensor(
    batch_size_, {output_image_width_, output_image_height_},
    output_image_format.interleaved_float_format);

  norm_tensor_ = nvcv::Tensor(
    batch_size_, {output_image_width_, output_image_height_},
    output_image_format.interleaved_float_format);
}

ImageEncoderNode::~ImageEncoderNode() = default;

void ImageEncoderNode::InputCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  size_t buffer_size{msg->step * msg->height};

  cudaMemcpy(input_image_buffer_.basePtr, msg->data.data(), buffer_size, cudaMemcpyDefault);

  // Resize
  resize_op_((cudaStream_t) 0, input_image_tensor_, resized_tensor_, NVCV_INTERP_LINEAR);

  NVCVImageFormat input_image_format = g_str_to_nvcv_image_format[input_image_encoding_];
  NVCVImageFormat output_image_format = g_str_to_nvcv_image_format[output_image_encoding_];
  // Convert the color encoding only if the input and output image encodings are different.
  if (input_image_format.interleaved_format != output_image_format.interleaved_format) {
    // cvtColor operation converts the image from input_image_encoding to
    // output_image_encoding
    cvtColor_op_(
      (cudaStream_t) 0, resized_tensor_,
      color_converted_tensor_, color_conversion_code_
    );

    // ConvertTo operation converts the uint8 image to float32 image type.
    // Also, it divides each pixel value by 255.0 i.e. scales the pixel values
    // in range [0.0, 1.0]
    convert_op_(
      (cudaStream_t) 0, color_converted_tensor_,
      float_tensor_, 1.0f / 255.f, 0.0f
    );
  } else {
    convert_op_((cudaStream_t) 0, resized_tensor_, float_tensor_, 1.0f / 255.f, 0.0f);
  }

  // Normalize
  uint32_t flags = CVCUDA_NORMALIZE_SCALE_IS_STDDEV;
  norm_op_(
    (cudaStream_t) 0, float_tensor_, mean_tensor_, std_dev_tensor_,
    norm_tensor_, 1.0f, 0.0f, 0.0f, flags
  );

  // Convert the data layout from interleaved to planar
  reformat_op_((cudaStream_t) 0, norm_tensor_, output_image_tensor_);

  // Create the output buffer and copy the data
  float * output_buffer;
  size_t output_buffer_size{
    output_image_width_ * output_image_height_ * output_image_channels_ * sizeof(float)};
  cudaMalloc(&output_buffer, output_buffer_size);

  cudaMemcpy(output_buffer, output_image_buffer_.basePtr, output_buffer_size, cudaMemcpyDefault);

  // Copy the header information.
  std_msgs::msg::Header header;
  header.stamp.sec = msg->header.stamp.sec;
  header.stamp.nanosec = msg->header.stamp.nanosec;
  header.frame_id = msg->header.frame_id;

  nvidia::isaac_ros::nitros::NitrosTensorList tensor_list =
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(header)
    .AddTensor(
    tensor_name_,
    (
      nvidia::isaac_ros::nitros::NitrosTensorBuilder()
      .WithShape({batch_size_, output_image_channels_, output_image_height_, output_image_width_})
      .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kFloat32)
      .WithData(output_buffer)
      .Build()
    )
    )
    .Build();

  nitros_pub_->publish(tensor_list);
}
}  // namespace custom_nitros_dnn_image_encoder

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_dnn_image_encoder::ImageEncoderNode)
