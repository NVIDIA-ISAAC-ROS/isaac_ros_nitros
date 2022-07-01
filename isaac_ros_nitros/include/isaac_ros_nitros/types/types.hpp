/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__TYPES_HPP_
#define ISAAC_ROS_NITROS__TYPES__TYPES_HPP_

// NitrosInt64
#include "isaac_ros_nitros/types/nitros_int64.hpp"

// NitrosTensorList
#include "isaac_ros_nitros/types/nitros_tensor_list.hpp"

// NitrosCameraInfo
#include "isaac_ros_nitros/types/nitros_camera_info.hpp"

// NitrosAprilTagDetectionArray
#include "isaac_ros_nitros/types/nitros_april_tag_detection_array.hpp"

// NitrosImage
#include "isaac_ros_nitros/types/nitros_image.hpp"

// NitrosPointCloud
#include "isaac_ros_nitros/types/nitros_point_cloud.hpp"

// NitrosDisparityImage
#include "isaac_ros_nitros/types/nitros_disparity_image.hpp"


#define FOREACH_NITROS_DATA_FORMAT(_) \
  _(nvidia::isaac_ros::nitros::nitros_int64_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_rgb_f32_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_bgr_f32_t) \
  _(nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_bgr_f32_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_rgb8_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_rgba8_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_rgb16_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_bgr8_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_bgra8_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_bgr16_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_mono8_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_mono16_t) \
  _(nvidia::isaac_ros::nitros::nitros_image_nv24_t) \
  _(nvidia::isaac_ros::nitros::nitros_camera_info_t) \
  _(nvidia::isaac_ros::nitros::nitros_disparity_image_32FC1_t) \
  _(nvidia::isaac_ros::nitros::nitros_disparity_image_bgr8_t) \
  _(nvidia::isaac_ros::nitros::nitros_point_cloud_t) \
  _(nvidia::isaac_ros::nitros::nitros_april_tag_detection_array_t)

#endif  // ISAAC_ROS_NITROS__TYPES__TYPES_HPP_
