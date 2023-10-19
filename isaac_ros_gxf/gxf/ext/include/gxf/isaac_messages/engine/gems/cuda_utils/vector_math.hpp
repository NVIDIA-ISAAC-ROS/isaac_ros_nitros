// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "cuda_runtime.h"

namespace nvidia {
namespace isaac {

inline __host__ __device__ float3 operator-(const float3& a, const float3& b) {
  return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline __host__ __device__ float2 operator-(const float2& a, const float2& b) {
  return make_float2(a.x - b.x, a.y - b.y);
}

inline __host__ __device__ float2 operator-(const float2& a, const int2& b) {
  return make_float2(a.x - static_cast<float>(b.x), a.y - static_cast<float>(b.y));
}

inline __host__ __device__ float2 operator-(const int2& a, const float2& b) {
  return make_float2(static_cast<float>(a.x) - b.x, static_cast<float>(a.y) - b.y);
}

inline __host__ __device__ int3 operator-(const int3& a, const int3& b) {
  return make_int3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline __host__ __device__ int2 operator-(const int2& a, const int2& b) {
  return make_int2(a.x - b.x, a.y - b.y);
}

inline __host__ __device__ float3 operator+(const float3& a, const float3& b) {
  return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline __host__ __device__ float2 operator+(const float2& a, const float2& b) {
  return make_float2(a.x + b.x, a.y + b.y);
}

inline __host__ __device__ int2 operator+(const int2& a, const int2& b) {
  return make_int2(a.x + b.x, a.y + b.y);
}

inline __host__ __device__ float3 operator*(const float3& a, const float& s) {
  return make_float3(s*a.x, s*a.y, s*a.z);
}

inline __host__ __device__ float3 operator*(const float& s, const float3& a) {
  return make_float3(s*a.x, s*a.y, s*a.z);
}

inline __host__ __device__ float2 operator*(const float2& a, const float& s) {
  return make_float2(s*a.x, s*a.y);
}

inline __host__ __device__ float2 operator*(const float& s, const float2& a) {
  return make_float2(s*a.x, s*a.y);
}

// Coeffient-wise multiplication for two 3-vectors
inline __host__ __device__ float3 CwiseMult(const float3& a, const float3& b) {
  return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
}

// Dot product for two float3 vectors
inline __host__ __device__ float Dot(const float3& a, const float3 b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

// Dot product for two float2 vectors
inline __host__ __device__ float Dot(const float2& a, const float2 b) {
  return a.x*b.x + a.y*b.y;
}

// Dot product for two int3 vectors
inline __host__ __device__ int Dot(const int3& a, const int3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

// Dot product for two int2 vectors
inline __host__ __device__ int Dot(const int2& a, const int2& b) {
  return a.x*b.x + a.y*b.y;
}

// Cross product for two float3 vectors
inline __host__ __device__ float3 Cross(const float3& a, const float3 b) {
  return make_float3(a.y * b.z - a.z * b.y,
                     a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x);
}

// Square of 2-norm for a float3 vector
inline __host__ __device__ float SquareNorm(const float3& a) {
  return Dot(a, a);
}

// Square of 2-norm for a int3 vector
inline __host__ __device__ int SquareNorm(const int3& a) {
  return Dot(a, a);
}

// Square of 2-norm for a float2 vector
inline __host__ __device__ float SquareNorm(const float2& a) {
  return Dot(a, a);
}

// 2-norm for a float3 vector
inline __host__ __device__ float Norm(const float3& a) {
  return sqrtf(SquareNorm(a));
}

// 2-norm for a float2 vector
inline __host__ __device__ float Norm(const float2& a) {
  return sqrtf(SquareNorm(a));
}

// Coefficient-wise sum for a float3 vector
inline __host__ __device__ float Sum(const float3& a) {
  return a.x + a.y + a.z;
}

inline __host__ __device__ int2 Floorf(const float2& a) {
  return make_int2(floorf(a.x), floorf(a.y));
}

// Coefficient-wise conversion to int for a uchar3 vector
inline __host__ __device__ int3 ToInt(uchar3 a) {
  return make_int3(static_cast<int>(a.x), static_cast<int>(a.y), static_cast<int>(a.z));
}

// Coefficient-wise conversion to float for a int3 vector
inline __host__ __device__ float3 ToFloat(int3 a) {
  return make_float3(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z));
}

// Coefficient-wise conversion to float for a int2 vector
inline __host__ __device__ float2 ToFloat(int2 a) {
  return make_float2(static_cast<float>(a.x), static_cast<float>(a.y));
}

// Accesses the index-th element of a float4 vector. Assumes index = 0 if index not in {0, 1, 2, 3}.
inline __device__ float& Coefficient(float4& a, int index) {
  if (index == 0) return a.x;
  else if (index == 1) return a.y;
  else if (index == 2) return a.z;
  else if (index == 3) return a.w;
  else return a.x;  // NOLINT
}

// Accesses the index-th element of a short4 vector. Assumes index = 0 if index not in {0, 1, 2, 3}.
inline __device__ short& Coefficient(short4& a, int index) {  // NOLINT
  if (index == 0) return a.x;
  else if (index == 1) return a.y;
  else if (index == 2) return a.z;
  else if (index == 3) return a.w;
  else return a.x;  // NOLINT
}

// Rotate a float2 unit vector by another float2 vector. The vector elements represent the cos and
// sin of the rotation angle of the vector.
inline __host__ __device__ float2 Rotate(const float2& a, const float2& b) {
  return make_float2(a.x * b.x - a.y * b.y, a.y * b.x + a.x * b.y);
}

}  // namespace isaac
}  // namespace nvidia
