/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_
#define DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_

#include <vector>

#include "gxf/core/entity.hpp"
#include "gxf/std/timestamp.hpp"
#include "detection2_d.hpp"

namespace nvidia
{
namespace isaac_ros
{

// This struct helps parse the data coming from a gxf message. The message entity consists of a
// `Detection2D` component and a `Timestamp` component.
// Note that we do not add the raw type but rather a `Handle` to the type.
struct Detection2DParts
{
  gxf::Entity message;
  gxf::Handle<std::vector<Detection2D>> detection2_d_array;
  gxf::Handle<gxf::Timestamp> timestamp;
};

// This function creates a new entity and adds the (default-initialized) components from the parts
// struct. It returns the created parts struct s.t. we can then use this to modify the data
// contained in the entity.
gxf::Expected<Detection2DParts> CreateDetection2DList(gxf_context_t context);

// This function allows to parse an entity and returns the parts struct such that we have easy
// access to the components.
gxf::Expected<Detection2DParts> GetDetection2DList(gxf::Entity message);

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // DETECTNET__DETECTION2_D_ARRAY_MESSAGE_HPP_
