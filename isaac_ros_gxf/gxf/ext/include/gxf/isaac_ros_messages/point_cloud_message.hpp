/*
Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGES_POINT_CLOUD_HPP_
#define NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGES_POINT_CLOUD_HPP_

#include "gems/common/pose_frame_uid.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac_ros {
namespace messages {

// Data structure holding meta information about flatscan messages. Every flatscan message must
// contain an instance of this message.
struct PointCloudInfo {
  // If the beam free range end is greater than or equal to this value the beam did not hit an
  // obstacle at the end of the free range.
  double max_range;
  // Beams with a range lesser than or equal to this range in meters are considered to have returned
  // an invalid measurement.
  double min_range;
  // Flag showing whether this pointcloud message contains color data
  bool use_color;
  // Is this data bigendian?
  bool is_bigendian;
};

// Stores the point cloud message and provides views to it.
struct PointCloudMessageParts {
  // The message entity
  gxf::Entity message;
  // Tensor containing the point positions(and color if use_color is true)
  // Each color channel data is stored as a float in the colors array.
  // Each point data bytes are ordered as:
  // x_pos_float, y_pos_float, z_pos_float, color_float(only if if use_color is true)
  // If use_color is true and is_bigendian is true
  // color_float -> uint8 Data format: B, G, R, dont_care_byte
  // Else if use_color is true and is_bigendian is false
  // color_float -> uint8 Data format: dont_care_byte, R, G, B
  gxf::Handle<gxf::Tensor> points;
  // Pointcloud info data
  gxf::Handle<PointCloudInfo> info;
  // The uid of this scan's origin pose frame
  gxf::Handle<nvidia::isaac::PoseFrameUid> pose_frame_uid;
  // Timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Allocates a message entity representing a point cloud message, i.e. the cartesian coordinates of
// points and color is enabled. The returned message is a struct of
// type 'PointCloudMessageParts', consisting of an entity holding all data and
// allowing easy access to individual message parts.
gxf::Expected<PointCloudMessageParts> CreatePointCloudMessage(
    gxf_context_t context, gxf::Handle<gxf::Allocator> allocator, int point_count, bool use_color);

// Returns a struct of type `PointCloudMessageParts`
// consisting of an entity holding all point cloud data.
gxf::Expected<PointCloudMessageParts> GetPointCloudMessage(gxf::Entity message);

}  // namespace messages
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_ROS_EXTENSIONS_MESSAGES_POINT_CLOUD_HPP_
