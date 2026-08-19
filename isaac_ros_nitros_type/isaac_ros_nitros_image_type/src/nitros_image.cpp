// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_image_type/nitros_image.hpp"

#include <cuda_runtime.h>
#include <vpi/CUDAInterop.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>

#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace
{
// Compute total bytes for various encodings, including multi-plane formats
static size_t nitros_compute_total_image_bytes(
  const std::string & encoding, uint32_t width, uint32_t height, uint32_t step)
{
  // Multi-plane encodings treated as contiguous Y plane followed by chroma plane
  // NV12: Y plane WxH, UV plane interleaved with size WxH/2
  if (encoding == "nv12") {
    const size_t y_bytes = static_cast<size_t>(step) * height;  // step is Y pitch
    const size_t uv_bytes = y_bytes / 2;
    return y_bytes + uv_bytes;
  }
  // NV24: Y plane WxH, UV plane interleaved with full resolution (2 bytes per pixel)
  if (encoding == "nv24") {
    const size_t y_bytes = static_cast<size_t>(step) * height;  // step is Y pitch
    const size_t uv_bytes = static_cast<size_t>(width) * height * 2;
    return y_bytes + uv_bytes;
  }
  // Fallback: assume tightly packed single-plane using provided step
  return static_cast<size_t>(step) * height;
}

// Convert multi-plane NV12/NV24 device memory to interleaved RGB8 host memory using VPI
static void convert_nv_to_rgb8(
  const nvidia::isaac_ros::nitros::NitrosImage & source,
  const uint8_t * dev_ptr,
  sensor_msgs::msg::Image & destination,
  cudaStream_t stream)
{
  const bool is_nv12 = (destination.encoding == "nv12");

  VPIStream vpi_stream{};
  VPIStatus st = vpiStreamCreateWrapperCUDA(stream, VPI_BACKEND_CUDA, &vpi_stream);
  if (st != VPI_SUCCESS) {
    throw std::runtime_error("vpiStreamCreateWrapperCUDA failed");
  }

  VPIImage input{}, output{};
  VPIImageData imgdata{};
  std::memset(&imgdata, 0, sizeof(VPIImageData));
  imgdata.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
  imgdata.buffer.pitch.numPlanes = 2;
  imgdata.buffer.pitch.format = is_nv12 ? VPI_IMAGE_FORMAT_NV12_ER : VPI_IMAGE_FORMAT_NV24_ER;

  const auto & planes = source.get_color_planes();
  imgdata.buffer.pitch.planes[0].pBase = const_cast<uint8_t *>(dev_ptr + planes[0].offset);
  imgdata.buffer.pitch.planes[0].height = planes[0].height;
  imgdata.buffer.pitch.planes[0].width = planes[0].width;
  imgdata.buffer.pitch.planes[0].pixelType = VPI_PIXEL_TYPE_U8;
  imgdata.buffer.pitch.planes[0].offsetBytes = 0;
  imgdata.buffer.pitch.planes[0].pitchBytes = planes[0].stride;
  imgdata.buffer.pitch.planes[1].pBase = const_cast<uint8_t *>(dev_ptr + planes[1].offset);
  imgdata.buffer.pitch.planes[1].height = planes[1].height;
  imgdata.buffer.pitch.planes[1].width = planes[1].width;
  imgdata.buffer.pitch.planes[1].pixelType = VPI_PIXEL_TYPE_2U8;
  imgdata.buffer.pitch.planes[1].offsetBytes = 0;
  imgdata.buffer.pitch.planes[1].pitchBytes = planes[1].stride;

  st = vpiImageCreateWrapper(&imgdata, nullptr, VPI_BACKEND_CUDA, &input);
  if (st != VPI_SUCCESS) {
    vpiStreamDestroy(vpi_stream);
    throw std::runtime_error("VPI image wrapper create failed");
  }

  st = vpiImageCreate(destination.width, destination.height, VPI_IMAGE_FORMAT_RGB8,
      VPI_BACKEND_CUDA, &output);
  if (st != VPI_SUCCESS) {
    vpiImageDestroy(input);
    vpiStreamDestroy(vpi_stream);
    throw std::runtime_error("VPI image create failed");
  }

  st = vpiSubmitConvertImageFormat(vpi_stream, VPI_BACKEND_CUDA, input, output, nullptr);
  if (st != VPI_SUCCESS) {
    vpiImageDestroy(output);
    vpiImageDestroy(input);
    vpiStreamDestroy(vpi_stream);
    throw std::runtime_error("VPI convert image format failed");
  }

  st = vpiStreamSync(vpi_stream);
  if (st != VPI_SUCCESS) {
    vpiImageDestroy(output);
    vpiImageDestroy(input);
    vpiStreamDestroy(vpi_stream);
    throw std::runtime_error("VPI stream sync failed");
  }

  VPIImageData outdata{};
  st = vpiImageLockData(output, VPI_LOCK_READ, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &outdata);
  if (st != VPI_SUCCESS) {
    vpiImageDestroy(output);
    vpiImageDestroy(input);
    vpiStreamDestroy(vpi_stream);
    throw std::runtime_error("VPI image lock failed");
  }

  destination.encoding = sensor_msgs::image_encodings::RGB8;
  destination.step = destination.width * 3;  // Tight packing for ROS message
  destination.data.resize(static_cast<size_t>(destination.step) * destination.height);

  const void * src_ptr = outdata.buffer.pitch.planes[0].pBase;
  size_t src_pitch = outdata.buffer.pitch.planes[0].pitchBytes;
  cudaError_t cuda_err = cudaMemcpy2DAsync(
    destination.data.data(), destination.step,
    src_ptr, src_pitch,
    destination.width * 3, destination.height,
    cudaMemcpyDeviceToHost, stream);
  if (cuda_err != cudaSuccess) {
    vpiImageUnlock(output);
    vpiImageDestroy(output);
    vpiImageDestroy(input);
    vpiStreamDestroy(vpi_stream);
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy2DAsync D2H failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  cuda_err = cudaStreamSynchronize(stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  (void)vpiImageUnlock(output);
  vpiImageDestroy(output);
  vpiImageDestroy(input);
  vpiStreamDestroy(vpi_stream);
}
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::Image>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  destination.height = source.height;
  destination.width = source.width;
  destination.encoding = source.encoding;
  destination.is_bigendian = 0;
  destination.step = source.step;
  const bool is_nv12 = (destination.encoding == "nv12");
  const bool is_nv24 = (destination.encoding == "nv24");

  auto stream_handle = nvidia::isaac_ros::nitros::CudaStreamPool::instance().get_stream_handle();
  cudaStream_t stream = stream_handle.get();

  auto read_handle = source.get_read_handle(stream);
  const uint8_t * dev_ptr = read_handle.get_ptr();

  if (dev_ptr == nullptr) {
    throw std::runtime_error("NitrosImage device pointer is nullptr");
  }

  if (is_nv12 || is_nv24) {
    convert_nv_to_rgb8(source, dev_ptr, destination, stream);
  } else {
    const size_t total_bytes = nitros_compute_total_image_bytes(
      destination.encoding, destination.width, destination.height, destination.step);
    destination.data.resize(total_bytes);
    cudaError_t cuda_err = cudaMemcpyAsync(
      destination.data.data(), dev_ptr, total_bytes,
      cudaMemcpyDeviceToHost, stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_ros_message] cudaMemcpyAsync D2H failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
    cuda_err = cudaStreamSynchronize(stream);
    if (cuda_err != cudaSuccess) {
      std::stringstream error_msg;
      error_msg <<
        "[convert_to_ros_message] cudaStreamSynchronize failed: " <<
        cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
      RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }

  destination.header.stamp.sec = source.timestamp_sec;
  destination.header.stamp.nanosec = source.timestamp_nsec;
  destination.header.frame_id = source.frame_id;
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::Image>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  const size_t bytes = nitros_compute_total_image_bytes(
    source.encoding, source.width, source.height, source.step);

  auto & stream_pool = nvidia::isaac_ros::nitros::CudaStreamPool::instance();
  cudaStream_t stream = stream_pool.acquire();

  nvidia::isaac_ros::nitros::NitrosImage msg_temp;
  uint8_t * dptr = nullptr;
  cudaError_t cuda_err = cudaMallocAsync(reinterpret_cast<void **>(&dptr), bytes, stream);
  if (cuda_err != cudaSuccess) {
    stream_pool.release(stream);
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMallocAsync failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto deleter = [&stream_pool, stream](uint8_t * p){
      if (p) {
        cudaFreeAsync(p, stream);
      }
      stream_pool.release(stream);
    };

  auto write_handle = msg_temp.from_external(
    dptr, bytes, source.width, source.height, source.step, source.encoding, stream, deleter);

  cuda_err = cudaMemcpyAsync(write_handle.get_ptr(), source.data.data(), bytes,
    cudaMemcpyHostToDevice, stream);
  if (cuda_err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpyAsync H2D failed: " <<
      cudaGetErrorName(cuda_err) << " (" << cudaGetErrorString(cuda_err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  destination = std::move(msg_temp);
  destination.width = source.width;
  destination.height = source.height;
  destination.step = source.step;
  destination.encoding = source.encoding;
  destination.timestamp_sec = source.header.stamp.sec;
  destination.timestamp_nsec = source.header.stamp.nanosec;
  destination.frame_id = source.header.frame_id;
  static_cast<nvidia::isaac_ros::nitros::NitrosTypeBase &>(destination)
  .frame_id = source.header.frame_id;
}
