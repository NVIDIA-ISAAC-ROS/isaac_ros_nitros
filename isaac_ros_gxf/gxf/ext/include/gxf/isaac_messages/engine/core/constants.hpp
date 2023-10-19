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

#include <type_traits>

namespace nvidia {
namespace isaac {

// The value of pi with 75 digits available for every type.
template <typename K,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
constexpr K Pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
// The value of 2 * pi (a.k.a tau) with 75 digits available for every type.
template <typename K,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
constexpr K TwoPi = 6.283185307179586476925286766559005768394338798750211641949889184615632812572;

// Converts from degree to radians
template <typename K,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
constexpr K DegToRad(K x) {
  return x * K(0.017453292519943295769236907684886127134428718885417254560);
}

// Converts from radians to degrees
template <typename K,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
constexpr K RadToDeg(K x) {
  return x * K(57.29577951308232087679815481410517033240547246656432154916);
}

}  // namespace isaac
}  // namespace nvidia
