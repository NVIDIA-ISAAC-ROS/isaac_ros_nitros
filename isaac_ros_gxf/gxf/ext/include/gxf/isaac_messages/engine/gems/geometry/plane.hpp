// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "Eigen/Eigen"

namespace nvidia {
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
}  // namespace nvidia
