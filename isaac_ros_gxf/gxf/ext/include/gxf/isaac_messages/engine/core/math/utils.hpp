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

#include <cmath>
#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/macros.hpp"
#include "engine/core/math/types.hpp"

namespace nvidia {
namespace isaac {

// Returns square of a number. This is alternative to using std::pow(number, 2).
template <typename K>
CUDA_BOTH K Square(K number) {
  return number * number;
}

// Returns an angle within the range ]-Pi, Pi] that is equivalent to `angle`:
//   WrapPi(angle) == angle + k*2*Pi (k being an integer)
template <typename K>
CUDA_BOTH K WrapPi(K angle) {
  // Use -angle in order to include Pi and exclude -Pi
  const K d = std::floor((Pi<K> - angle) / TwoPi<K>);
  return angle + d * TwoPi<K>;
}

// Returns an angle within the range [0, 2*Pi[ that is equivalent to `angle`:
//   WrapTwoPi(angle) == angle + k*2*Pi (k being an integer)
template <typename K>
CUDA_BOTH K WrapTwoPi(K angle) {
  while (angle >= TwoPi<K>) angle -= TwoPi<K>;
  while (angle < K(0)) angle += TwoPi<K>;
  return angle;
}

// Returns the difference between two angles in the range ]-Pi, Pi]
// Warning to not use the result to compare to zero if the angles are big.
template <typename K>
CUDA_BOTH K DeltaAngle(K x, K y) {
  return WrapPi(x - y);
}

// Returns the average of two angles in the range ]-Pi, Pi]
template <typename K>
CUDA_BOTH K MeanAngle(K x, K y) {
  return WrapPi(y + 0.5 * DeltaAngle(x, y));
}

// Computes a unit vector at given angle (anti clock-wise from x-axis)
template<typename K>
CUDA_BOTH Vector2<K> AngleVector(K angle) {
  return Vector2<K>{std::cos(angle), std::sin(angle)};
}

// Limits the absolute of a value
template <typename K>
CUDA_BOTH K LimitAbs(K x, K max) {
  return x < -max ? -max : (x > max ? max : x);
}

// Clamps a value to the given interval [xmin, xmax]
template <typename K>
CUDA_BOTH K Clamp(K x, K xmin, K xmax) {
  return x < xmin ? xmin : (x > xmax ? xmax : x);
}
// Clamps a value to the unit interval [0, 1]
template <typename K>
CUDA_BOTH K Clamp01(K x) {
  return Clamp(x, K(0), K(1));
}

// Safely cast a floating point number into an integral type and clamp the result in the range
// [min, max]. If NaN is provided as input, then max will be returned.
template <typename K, typename I,
          typename std::enable_if_t<std::is_floating_point<K>::value, int> = 0,
          typename std::enable_if_t<std::is_integral<I>::value, int> = 0>
CUDA_BOTH I SafeClampAndCast(K value, I min, I max) {
  if (value < static_cast<K>(min)) {
    return min;
  }
  if (value < static_cast<K>(max)) {
    return static_cast<I>(value);
  }
  return max;
}

// Convenience function which calls std::floor and stores the result in a 32-bit integer.
template <typename K,
          typename std::enable_if_t<std::is_floating_point<K>::value, int> = 0>
CUDA_BOTH int FloorToInt(K value) {
  return static_cast<int>(std::floor(value));
}

// Rounds a double to a 32-bit integer using a nifty trick. This function only works if the double
// is within reasonable range.
CUDA_BOTH_INLINE int FastRoundDoubleToInt32(double x) {
  union {
    double real;
    int integer;
  } q;
  q.real = x + 6755399441055744.0;
  return q.integer;
}

// Convenience function which calls std::floor and stores the result in a 32-bit integer.
template <typename K,
          typename std::enable_if_t<std::is_floating_point<K>::value, int> = 0>
CUDA_BOTH int FloorToIntWithReminder(K value, K& reminder) {
  const K floor_value = std::floor(value);
  reminder = value - floor_value;
  return static_cast<int>(floor_value);
}

// Computes 0 <= a such that k*n + a = x for 0 <= k < n
CUDA_BOTH_INLINE int PositiveModulo(int x, int n) {
  const int k = x % n;
  return (k < 0 ? k + n : k);
}

// Computes smallest q such that q * b >= a for a >= 0 and b > 0. If a or b are out of bounds
// -1 is returned.
CUDA_BOTH_INLINE int CeilDivision(int a, int b) {
  if (a < 0 || b <= 0) return -1;
  return (a / b) + (a % b != 0 ? 1 : 0);  // Note the assumption is that / and % are computed
                                          // as part of the same operation.
}

// Returns whether or not two vectors are colinear
template<typename K, int N>
CUDA_BOTH bool IsColinear(const Vector<K, N>& a, const Vector<K, N>& b) {
  const K dot = a.dot(b);
  // If dot is null, either one of the vector is null which means the two vectors are colinear, if
  // not then the two vectors are orthogonal so obviously not colinear.
  if (IsAlmostZero(dot)) {
    return IsAlmostZero(a.squaredNorm()) || IsAlmostZero(b.squaredNorm());
  }
  // a.dot(b) / (||a|| * ||b||) = cos(alpha) where alpha is the angle between both vector;
  // Taking the square of it avoid the use of sqrt and both vector are colinear iff |cos(alpha)| = 1
  return IsAlmostOne(dot * dot / (a.squaredNorm() * b.squaredNorm()));
}

// Returns the cross product of Vector2 defined as:
// AxB = ||A||*||B||*sin(A/B);
template<typename K>
CUDA_BOTH K CrossProduct(const Vector2<K>& a, const Vector2<K>& b) {
  return a.x() * b.y() - a.y() * b.x();
}

}  // namespace isaac
}  // namespace nvidia
