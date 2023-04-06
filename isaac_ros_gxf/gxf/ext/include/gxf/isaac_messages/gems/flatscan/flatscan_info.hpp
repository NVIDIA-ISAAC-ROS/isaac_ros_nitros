/*
Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

namespace nvidia {
namespace isaac {

// Data structure holding meta information about flatscan messages. Every flatscan message must
// contain an instance of this message.
struct FlatscanInfo {
  // If the beam free range end is greater than or equal to this value the beam did not hit an
  // obstacle at the end of the free range.
  double out_of_range;
  // Horizontal width of every beam in radians.
  double beam_width;
  // The lower z value of the height slice.
  double height_min;
  // The upper z value of the height slice.
  double height_max;
};

}  // namespace isaac
}  // namespace nvidia
