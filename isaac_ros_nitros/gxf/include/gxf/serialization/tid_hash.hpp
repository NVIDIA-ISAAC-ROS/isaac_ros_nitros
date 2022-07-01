/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_
#define NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_

#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

// Hash function for gxf_tid_t that XORs upper 64 bits with lower 64 bits
struct TidHash {
  size_t operator()(const gxf_tid_t& tid) const { return tid.hash1 ^ tid.hash2; }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_TID_HASH_HPP_
