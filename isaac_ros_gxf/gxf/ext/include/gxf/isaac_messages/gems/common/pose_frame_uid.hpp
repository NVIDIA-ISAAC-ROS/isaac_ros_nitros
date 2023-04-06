/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_ISAAC_GEMS_COMMON_POSE_FRAME_UID_HPP_
#define NVIDIA_ISAAC_GEMS_COMMON_POSE_FRAME_UID_HPP_

#include <cstdint>

namespace nvidia {
namespace isaac {

// Spatial information for the data in a message.
struct PoseFrameUid {
  // Unique identifier of the pose tree frame to which this data is attached.
  uint64_t uid;
};

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_GEMS_COMMON_POSE_FRAME_UID_HPP_
