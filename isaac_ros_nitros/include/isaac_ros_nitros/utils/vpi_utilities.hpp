/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__UTILS__VPI_UTILITIES_HPP_
#define ISAAC_ROS_NITROS__UTILS__VPI_UTILITIES_HPP_

#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/multimedia/video.hpp"
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "vpi/VPI.h"

// VPI status check macro
#define CHECK_VPI_STATUS(STMT) \
  do { \
    VPIStatus status = (STMT); \
    if (status != VPI_SUCCESS) { \
      char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH]; \
      vpiGetLastStatusMessage(buffer, sizeof(buffer)); \
      std::ostringstream ss; \
      ss << __FILE__ << ":" << __LINE__ << ": " << vpiStatusGetName(status) << ": " << buffer; \
      RCLCPP_ERROR( \
        rclcpp::get_logger( \
          "Isaac ROS Common VPI utilities"), "Error in VPI method. Error[%s]", ss.str().c_str()); \
      throw std::runtime_error("Error in VPI method."); \
    } \
  } while (0);

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

/**
 * @brief Declare and parse ROS 2 parameter into VPI backend flags
 *
 * @param node The node to declare the parameter with
 * @param default_backends The default backends to use if given invalid input
 * @return uint32_t The resulting VPI backend flags
 */
uint32_t DeclareVPIBackendParameter(rclcpp::Node * node, uint32_t default_backends) noexcept;

/**
 * @brief Data structure to hold VPI format information
 *
 */
struct VPIFormat
{
  VPIImageFormat image_format;
  std::vector<VPIPixelType> pixel_type;
};

using VideoFormat = nvidia::gxf::VideoFormat;
/**
 * @brief Convert a GXF video format into VPI format
 *
 * @param value Input GXF VideoFormat
 * @return VPIFormat Resulting VPIFormat
 */
VPIFormat ToVpiFormat(VideoFormat value);

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__UTILS__VPI_UTILITIES_HPP_
