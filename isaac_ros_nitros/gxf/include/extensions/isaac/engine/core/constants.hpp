/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <type_traits>

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
