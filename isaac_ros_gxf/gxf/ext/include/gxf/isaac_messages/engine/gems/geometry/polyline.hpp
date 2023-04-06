/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/math/types.hpp"

namespace isaac {
namespace geometry {

// A polyline is currently simply a list of points
template <typename K, int N>
using Polyline = std::vector<Vector<K, N>>;

using Polyline2d = Polyline<double, 2>;
using Polyline2f = Polyline<float, 2>;
using Polyline3d = Polyline<double, 3>;
using Polyline3f = Polyline<float, 3>;

}  // namespace geometry
}  // namespace isaac
