/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <cuda_runtime.h>

#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/video.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "isaac_ros_nitros/utils/vpi_utilities.hpp"
#include "vpi/Image.h"
#include "vpi/algo/ConvertImageFormat.h"

constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

namespace
{

using VideoFormat = nvidia::gxf::VideoFormat;
namespace img_encodings = sensor_msgs::image_encodings;
namespace nitros = nvidia::isaac_ros::nitros;
// Map to store the ROS format encoding to Nitros format encoding
const std::unordered_map<std::string, VideoFormat> g_ros_to_gxf_video_format({
    {img_encodings::RGB8, VideoFormat::GXF_VIDEO_FORMAT_RGB},
    {img_encodings::RGBA8, VideoFormat::GXF_VIDEO_FORMAT_RGBA},
    {img_encodings::RGB16, VideoFormat::GXF_VIDEO_FORMAT_RGB16},
    {img_encodings::BGR8, VideoFormat::GXF_VIDEO_FORMAT_BGR},
    {img_encodings::BGRA8, VideoFormat::GXF_VIDEO_FORMAT_BGRA},
    {img_encodings::BGR16, VideoFormat::GXF_VIDEO_FORMAT_BGR16},
    {img_encodings::MONO8, VideoFormat::GXF_VIDEO_FORMAT_GRAY},
    {img_encodings::MONO16, VideoFormat::GXF_VIDEO_FORMAT_GRAY16},
    {img_encodings::NV24, VideoFormat::GXF_VIDEO_FORMAT_NV24_ER},
    {"nv12", VideoFormat::GXF_VIDEO_FORMAT_NV12_ER},
  });

// Map to store the Nitros format encoding to ROS format encoding
const std::unordered_map<VideoFormat, std::string> g_gxf_to_ros_video_format({
    {VideoFormat::GXF_VIDEO_FORMAT_RGB, img_encodings::RGB8},
    {VideoFormat::GXF_VIDEO_FORMAT_RGBA, img_encodings::RGBA8},
    {VideoFormat::GXF_VIDEO_FORMAT_RGB16, img_encodings::RGB16},
    {VideoFormat::GXF_VIDEO_FORMAT_BGR, img_encodings::BGR8},
    {VideoFormat::GXF_VIDEO_FORMAT_BGRA, img_encodings::BGRA8},
    {VideoFormat::GXF_VIDEO_FORMAT_BGR16, img_encodings::BGR16},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY, img_encodings::MONO8},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY16, img_encodings::MONO16},
    {VideoFormat::GXF_VIDEO_FORMAT_NV24_ER, img_encodings::NV24},
    {VideoFormat::GXF_VIDEO_FORMAT_NV12_ER, "nv12"},
  });

// Get step size for ROS Image
uint32_t get_step_size(const nvidia::gxf::VideoBufferInfo & video_buff_info)
{
  return video_buff_info.width * video_buff_info.color_planes[0].bytes_per_pixel;
}

uint32_t get_step_size(const VPIImageData & vpi_img_data)
{
  return vpi_img_data.buffer.pitch.planes[0].pitchBytes;
}

template<VideoFormat T>
struct NoPaddingColorPlanes {};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGB>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGB", 3, width * 3)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGR>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGR", 3, width * 3)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGBA>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGBA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGRA>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGRA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGB16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGB16", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGR16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGR16", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_GRAY>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("GRAY", 1, width)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_GRAY16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("GRAY", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_NV12>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("Y", 1, width),
        nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_NV24>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("Y", 1, width),
        nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};

template<VideoFormat T>
void allocate_video_buffer_no_padding(
  const uint32_t width,
  const uint32_t height,
  const nvidia::gxf::Handle<nvidia::gxf::VideoBuffer> & video_buff,
  const nvidia::gxf::Handle<nvidia::gxf::Allocator> & allocator_handle)
{
  constexpr auto surface_layout = nvidia::gxf::SurfaceLayout::GXF_SURFACE_LAYOUT_PITCH_LINEAR;
  constexpr auto storage_type = nvidia::gxf::MemoryStorageType::kDevice;
  if (width % 2 != 0 || height % 2 != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"),
      "[convert_to_custom] Image width/height must be even for creation of gxf::VideoBuffer");
    throw std::runtime_error("[convert_to_custom] Odd Image width or height.");
  }
  NoPaddingColorPlanes<T> nopadding_planes(width);
  nvidia::gxf::VideoFormatSize<T> format_size;
  uint64_t size = format_size.size(width, height, nopadding_planes.planes);
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImage"),
    "[image size]  [%ld].", size);
  std::vector<nvidia::gxf::ColorPlane> color_planes{nopadding_planes.planes.begin(),
    nopadding_planes.planes.end()};
  nvidia::gxf::VideoBufferInfo buffer_info{static_cast<uint32_t>(width),
    static_cast<uint32_t>(height),
    T, color_planes,
    surface_layout};

  video_buff->resizeCustom(buffer_info, size, storage_type, allocator_handle);
}

void allocate_video_buffer(
  const sensor_msgs::msg::Image & source,
  const nvidia::gxf::Handle<nvidia::gxf::VideoBuffer> & video_buff,
  const nvidia::gxf::Handle<nvidia::gxf::Allocator> & allocator_handle)
{
  auto color_fmt = g_ros_to_gxf_video_format.find(source.encoding);
  if (color_fmt == std::end(g_ros_to_gxf_video_format)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"),
      "[convert_to_custom] Unsupported encoding from ROS [%s].", source.encoding.c_str());
    throw std::runtime_error("[convert_to_custom] Unsupported encoding from ROS.");
  }

  switch (color_fmt->second) {
    case VideoFormat::GXF_VIDEO_FORMAT_RGB:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_RGB>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGBA:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_RGBA>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGB16:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_RGB16>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_BGR>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGRA:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_BGRA>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR16:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_BGR16>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_GRAY>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY16:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_GRAY16>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_NV24:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_NV24>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_NV12:
      allocate_video_buffer_no_padding<VideoFormat::GXF_VIDEO_FORMAT_NV12>(
        source.width, source.height, video_buff, allocator_handle);
      break;

    default:
      break;
  }
}

VPIStatus CreateVPIImageWrapper(
  VPIImage & vpi_image, VPIImageData & img_data, uint64_t flags,
  const nvidia::gxf::Handle<nvidia::gxf::VideoBuffer> & video_buff)
{
  nvidia::gxf::VideoBufferInfo image_info = video_buff->video_frame_info();
  nitros::VPIFormat vpi_format = nitros::ToVpiFormat(image_info.color_format);
  img_data.bufferType = VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR;
  img_data.buffer.pitch.format = vpi_format.image_format;
  img_data.buffer.pitch.numPlanes = image_info.color_planes.size();
  auto data_ptr_offset = 0;
  for (size_t i = 0; i < image_info.color_planes.size(); ++i) {
    img_data.buffer.pitch.planes[i].data = video_buff->pointer() + data_ptr_offset;
    img_data.buffer.pitch.planes[i].height = image_info.color_planes[i].height;
    img_data.buffer.pitch.planes[i].width = image_info.color_planes[i].width;
    img_data.buffer.pitch.planes[i].pixelType = vpi_format.pixel_type[i];
    img_data.buffer.pitch.planes[i].pitchBytes = image_info.color_planes[i].stride;

    data_ptr_offset = image_info.color_planes[i].size;
  }
  return vpiImageCreateWrapper(&img_data, nullptr, flags, &vpi_image);
}
}  // namespace


void rclcpp::TypeAdapter<nitros::NitrosImage, sensor_msgs::msg::Image>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nitros::nvtxRangePushWrapper("NitrosImage::convert_to_ros_message", nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImage"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto gxf_video_buffer = msg_entity->get<nvidia::gxf::VideoBuffer>();
  if (!gxf_video_buffer) {
    std::string error_msg =
      "[convert_to_ros_message] Failed to get the existing VideoBuffer object";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.c_str());
    throw std::runtime_error(error_msg.c_str());
  }

  // Setting Image date from gxf VideoBuffer
  auto video_buffer_info = gxf_video_buffer.value()->video_frame_info();
  destination.height = video_buffer_info.height;
  destination.width = video_buffer_info.width;
  const auto encoding = g_gxf_to_ros_video_format.find(video_buffer_info.color_format);
  if (encoding == std::end(g_gxf_to_ros_video_format)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"),
      "[convert_to_ros_message] Unsupported encoding from gxf [%d].",
      (int)video_buffer_info.color_format);
    throw std::runtime_error("[convert_to_custom] Unsupported encoding from gxf .");
  } else {
    destination.encoding = encoding->second;
  }

  void * src_ptr;
  size_t src_pitch{0};
  uint32_t step_size{0};
  VPIStream vpi_stream{};
  uint64_t vpi_flags{VPI_BACKEND_CUDA};
  VPIImage input{}, output{};
  VPIImageData input_data{}, output_data{};

  if (destination.encoding == "nv12" || destination.encoding == "nv24") {
    // Convert multiplanar to interleaved rgb
    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosImage"),
      "[convert_to_ros_message] Multiplanar to interleaved started");

    // Create VPI stream
    CHECK_VPI_STATUS(vpiStreamCreate(vpi_flags, &vpi_stream));

    // VPI input image
    CHECK_VPI_STATUS(CreateVPIImageWrapper(input, input_data, vpi_flags, gxf_video_buffer.value()));
    // VPI output image
    CHECK_VPI_STATUS(
      vpiImageCreate(
        destination.width, destination.height, VPI_IMAGE_FORMAT_RGB8, vpi_flags, &output));

    // Call for color format conversion
    CHECK_VPI_STATUS(vpiSubmitConvertImageFormat(vpi_stream, vpi_flags, input, output, nullptr));

    // Wait for operations to complete
    CHECK_VPI_STATUS(vpiStreamSync(vpi_stream));

    // Copy data
    CHECK_VPI_STATUS(
      vpiImageLockData(output, VPI_LOCK_READ, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &output_data));
    // Release lock
    CHECK_VPI_STATUS(vpiImageUnlock(output));

    destination.encoding = img_encodings::RGB8;
    src_ptr = output_data.buffer.pitch.planes[0].data;
    src_pitch = get_step_size(output_data);
    step_size = get_step_size(output_data);

    RCLCPP_DEBUG(
      rclcpp::get_logger("NitrosImage"),
      "[convert_to_ros_message] Multiplanar to interleaved finished");
  } else {
    src_ptr = gxf_video_buffer.value()->pointer();
    src_pitch = video_buffer_info.color_planes[0].stride;
    step_size = get_step_size(video_buffer_info);
  }

  destination.is_bigendian = 0;

  // Full row length in bytes
  destination.step = step_size;

  // Resize the ROS image buffer to the right size
  destination.data.resize(destination.step * destination.height);

  // Copy data from Device to Host
  const cudaError_t cuda_error = cudaMemcpy2D(
    destination.data.data(),
    destination.step,
    src_ptr,
    src_pitch,
    destination.step,
    destination.height,
    cudaMemcpyDeviceToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy2D failed for conversion from "
      "NitrosImage to sensor_msgs::msg::Image: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  vpiImageDestroy(input);
  vpiImageDestroy(output);
  vpiStreamDestroy(vpi_stream);

  // Populate timestamp information back into ROS header
  auto input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!input_timestamp) {    // Fallback to any 'timestamp'
    input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  }
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImage"),
    "[convert_to_ros_message] Conversion completed for handle=%ld", source.handle);

  nitros::nvtxRangePopWrapper();
}


void rclcpp::TypeAdapter<nitros::NitrosImage, sensor_msgs::msg::Image>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  nitros::nvtxRangePushWrapper("NitrosImage::convert_to_custom", nitros::CLR_PURPLE);

  RCLCPP_DEBUG(rclcpp::get_logger("NitrosImage"), "[convert_to_custom] Conversion started");

  auto context = nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  gxf_uid_t cid;
  nitros::GetTypeAdapterNitrosContext().getCid(
    kEntityName, kComponentName, kComponentTypeName, cid);

  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  auto message = nvidia::gxf::Entity::New(context);
  if (!message) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  auto gxf_video_buffer = message->add<nvidia::gxf::VideoBuffer>(source.header.frame_id.c_str());
  if (!gxf_video_buffer) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to create a VideoBuffer object: " <<
      GxfResultStr(gxf_video_buffer.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // The uint64_t component which represents as sequential frame number
  // is required by gxf camera_message
  auto uint64_t_buffer = message->add<uint64_t>();
  if (!uint64_t_buffer) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to create a uint64_t object: " <<
      GxfResultStr(uint64_t_buffer.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Allocate buffer
  allocate_video_buffer(source, gxf_video_buffer.value(), allocator_handle);

  auto video_buffer_info = gxf_video_buffer.value()->video_frame_info();

  // Copy data from Host to Device
  auto width = get_step_size(video_buffer_info);
  const cudaError_t cuda_error = cudaMemcpy2D(
    gxf_video_buffer.value()->pointer(),
    video_buffer_info.color_planes[0].stride,
    source.data.data(),
    source.step,
    width,
    video_buffer_info.height,
    cudaMemcpyHostToDevice);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy2D failed for conversion from "
      "sensor_msgs::msg::Image to NitrosImage: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  auto output_timestamp = message->add<nvidia::gxf::Timestamp>("timestamp");
  if (!output_timestamp) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to add a timestamp component to Image message: " <<
      GxfResultStr(output_timestamp.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImage"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosImage"),
    "[convert_to_custom] Conversion completed (resulting handle=%ld)", message->eid());

  nitros::nvtxRangePopWrapper();
}

// Map to store the Nitros image format encoding to Video buffer format
const std::unordered_map<std::string, VideoFormat> g_nitros_to_gxf_video_format({
  {"nitros_image_rgb8", VideoFormat::GXF_VIDEO_FORMAT_RGB},
  {"nitros_image_rgba8", VideoFormat::GXF_VIDEO_FORMAT_RGBA},
  {"nitros_image_rgb16", VideoFormat::GXF_VIDEO_FORMAT_RGB16},
  {"nitros_image_bgr8", VideoFormat::GXF_VIDEO_FORMAT_BGR},
  {"nitros_image_bgra8", VideoFormat::GXF_VIDEO_FORMAT_BGRA},
  {"nitros_image_bgr16", VideoFormat::GXF_VIDEO_FORMAT_BGR16},
  {"nitros_image_mono8", VideoFormat::GXF_VIDEO_FORMAT_GRAY},
  {"nitros_image_mono16", VideoFormat::GXF_VIDEO_FORMAT_GRAY16},
  {"nitros_image_nv12", VideoFormat::GXF_VIDEO_FORMAT_NV12},
  {"nitros_image_nv24", VideoFormat::GXF_VIDEO_FORMAT_NV24}
});

uint64_t calculate_image_size(const std::string image_type, uint32_t width, uint32_t height)
{
  auto color_fmt = g_nitros_to_gxf_video_format.find(image_type);
  if (color_fmt == std::end(g_nitros_to_gxf_video_format)) {
    throw std::runtime_error("[calculate_image_size] Unsupported encoding from ROS.");
  }

  uint64_t image_size = 0;
  switch (color_fmt->second) {
    case VideoFormat::GXF_VIDEO_FORMAT_RGB:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_RGB> format_size_rgb8;
      image_size = format_size_rgb8.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGBA:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_RGBA> format_size_rgba;
      image_size = format_size_rgba.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_RGB16:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_RGB16> format_size_rgb16;
      image_size = format_size_rgb16.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_BGR> format_size_bgr8;
      image_size = format_size_bgr8.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGRA:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_BGRA> format_size_bgra;
      image_size = format_size_bgra.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_BGR16:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_BGR16> format_size_bgr16;
      image_size = format_size_bgr16.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_GRAY> format_size_gray;
      image_size = format_size_gray.size(width, height);
      break;

    case VideoFormat::GXF_VIDEO_FORMAT_GRAY16:
      nvidia::gxf::VideoFormatSize<VideoFormat::GXF_VIDEO_FORMAT_GRAY16> format_size_gray16;
      image_size = format_size_gray16.size(width, height);
      break;

    default:
      break;
  }
  return image_size;
}
