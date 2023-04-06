/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>
#include <cstdint>

// This file provides functions to help comparing floating point numbers for equality.
// You should have a look at this excellent article for more details:
// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/

namespace isaac {

// A very small floating point number close to what can be represented by the corresponding
// floating point standard.
// For more details see https://en.wikipedia.org/wiki/Machine_epsilon
template<typename K>
constexpr K MachineEpsilon = K(0);
template<>
constexpr float MachineEpsilon<float> = 5.9604644775390625e-8f;
template<>
constexpr double MachineEpsilon<double> = 1.1102230246251565404236316680908203125e-16;

// Returns true if a floating point value can be considered to be zero under floating point
// rounding errors. This function compares against a custom epsilon.
template<typename K>
bool IsAlmostZero(K x, K epsilon = MachineEpsilon<K>) {
  return std::abs(x) <= epsilon;
}
// Returns true if two floating point values are so close that they can be considered to be equal
// under floating point rounding errors. This function uses relative comparison technique.
// Warning: Do not use to compare against small floats, or zero, use IsAlmostZero instead.
template<typename K>
bool IsAlmostEqualRelative(K x, K y, K max_rel_diff = K(10)*MachineEpsilon<K>) {
  const K diff = std::abs(x - y);
  const K absx = std::abs(x);
  const K absy = std::abs(y);
  const K larger = (absx > absy) ? absx : absy;
  return (diff <= larger * max_rel_diff);
}

// Returns true if two floating point values are equal up to a certain tolerance.
template <typename K>
bool IsAlmostEqualAbsolute(K x, K y, K tolerance) {
  return std::abs(x - y) <= tolerance;
}

// Returns true if a floating point value can be considered to be one under floating point
// rounding errors. This function compares against machine epsilon.
template<typename K>
bool IsAlmostOne(K x) {
  return std::abs(x - K(1)) <= K(10) * MachineEpsilon<K>;
}


namespace epsilon_details {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

inline int32_t Float32AsInteger(float x) {
  return *reinterpret_cast<int32_t*>(&x);
}

inline int64_t Float64AsInteger(double x) {
  return *reinterpret_cast<int64_t*>(&x);
}

#pragma GCC diagnostic pop
}  // namespace epsilon_details

// Checks if two 32-bit floats are equal by computing their ULPs difference.
inline bool IsAlmostEqualUlps(float x, float y, int32_t max_ulps_diff = 2) {
  const int32_t ix = epsilon_details::Float32AsInteger(x);
  const int32_t iy = epsilon_details::Float32AsInteger(y);
  if ((ix < 0) != (iy < 0)) {
    // In case the sign is different we still need to check if the floats were equal to
    // make sure -0 is equal to +0.
    return (x == y);
  } else {
    return std::abs(ix - iy) < max_ulps_diff;
  }
}

// Checks if two 64-bit floats are equal by computing their ULPs difference.
inline bool IsAlmostEqualUlps(double x, double y, int64_t max_ulps_diff = 2) {
  const int64_t ix = epsilon_details::Float64AsInteger(x);
  const int64_t iy = epsilon_details::Float64AsInteger(y);
  if ((ix < 0) != (iy < 0)) {
    // In case the sign is different we still need to check if the floats were equal to
    // make sure -0 is equal to +0.
    return (x == y);
  } else {
    return std::abs(ix - iy) < max_ulps_diff;
  }
}

}  // namespace isaac
