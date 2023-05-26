/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "Eigen/Eigen"

namespace isaac {
namespace geometry {

// A plane in N dimensions represented by a normal vector and an offset.
template<typename K, int N>
using Hyperplane = class Eigen::Hyperplane<K, N>;

template<typename K>
using Plane = Hyperplane<K, 3>;
using PlaneF = Plane<float>;
using PlaneD = Plane<double>;

}  // namespace geometry
}  // namespace isaac
