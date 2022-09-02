/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <Eigen/Eigen>

namespace isaac {

template <typename K, int N, int M>
using Matrix = Eigen::Matrix<K, N, M>;
template <typename K, int N>
using Vector = Eigen::Matrix<K, N, 1>;

template <typename K, int N>
using RowVector = Eigen::Matrix<K, 1, N>;
template <typename K, int N>
using ColVector = Eigen::Matrix<K, N, 1>;

template <typename K>
using VectorX = Vector<K, Eigen::Dynamic>;
using VectorXd = VectorX<double>;
using VectorXf = VectorX<float>;
using VectorXi = VectorX<int>;
using VectorXub = VectorX<uint8_t>;

using VectorXcf = Eigen::VectorXcf;

template <typename K>
using RowVectorX = RowVector<K, Eigen::Dynamic>;
using RowVectorXf = RowVectorX<float>;

template <typename K>
using ColVectorX = ColVector<K, Eigen::Dynamic>;
using ColVectorXf = ColVectorX<float>;

template <typename K>
using MatrixX = Matrix<K, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixXd = MatrixX<double>;
using MatrixXf = MatrixX<float>;
using MatrixXi = MatrixX<int>;

using MatrixXcf = Eigen::MatrixXcf;

#define DEFINE_MATRIX_TYPES(N)            \
  template <typename K>                   \
  using Matrix##N = Matrix<K, N, N>;      \
  using Matrix##N##d = Matrix##N<double>; \
  using Matrix##N##f = Matrix##N<float>;  \
  using Matrix##N##i = Matrix##N<int>;

DEFINE_MATRIX_TYPES(2)
DEFINE_MATRIX_TYPES(3)
DEFINE_MATRIX_TYPES(4)
DEFINE_MATRIX_TYPES(5)
DEFINE_MATRIX_TYPES(6)
DEFINE_MATRIX_TYPES(7)
DEFINE_MATRIX_TYPES(8)

#undef DEFINE_MATRIX_TYPES

// Matrix types with fixed number of rows but dynamic number of columns.
// Useful to store and efficiently manipulate sets of geometric entities, e.g. points and planes.
// 2xN: 2D points in Euclidean coordinates
// 3xN: 2D points in homogeneous coordinates or 3D points in Euclidean coordinates
// 4xN: planes or 3D points in homogeneous coordinates
#define DEFINE_FIXED_ROWS_MATRIX_TYPES(N) \
  template <typename K> using Matrix##N##X = Matrix<K, N, Eigen::Dynamic>; \
  using Matrix##N##Xd = Matrix##N##X<double>; \
  using Matrix##N##Xf = Matrix##N##X<float>; \
  using Matrix##N##Xi = Matrix##N##X<int>; \

DEFINE_FIXED_ROWS_MATRIX_TYPES(2)
DEFINE_FIXED_ROWS_MATRIX_TYPES(3)
DEFINE_FIXED_ROWS_MATRIX_TYPES(4)
DEFINE_FIXED_ROWS_MATRIX_TYPES(5)
DEFINE_FIXED_ROWS_MATRIX_TYPES(6)
DEFINE_FIXED_ROWS_MATRIX_TYPES(7)
DEFINE_FIXED_ROWS_MATRIX_TYPES(8)

#undef DEFINE_FIXED_ROWS_MATRIX_TYPES

template <typename K>
using Matrix34 = Matrix<K, 3, 4>;
using Matrix34d = Matrix34<double>;
using Matrix34f = Matrix34<float>;
using Matrix34i = Matrix34<int>;

template <typename K>
using Matrix43 = Matrix<K, 4, 3>;
using Matrix43d = Matrix43<double>;
using Matrix43f = Matrix43<float>;
using Matrix43i = Matrix43<int>;

#define DEFINE_VECTOR_TYPES(N)            \
  template <typename K>                   \
  using Vector##N = Vector<K, N>;         \
  using Vector##N##d = Vector##N<double>; \
  using Vector##N##f = Vector##N<float>;  \
  using Vector##N##i = Vector##N<int>;    \
  using Vector##N##ub = Vector##N<uint8_t>;

DEFINE_VECTOR_TYPES(2)
DEFINE_VECTOR_TYPES(3)
DEFINE_VECTOR_TYPES(4)
DEFINE_VECTOR_TYPES(5)
DEFINE_VECTOR_TYPES(6)
DEFINE_VECTOR_TYPES(7)
DEFINE_VECTOR_TYPES(8)

#undef DEFINE_VECTOR_TYPES

template <typename K>
using Quaternion = Eigen::Quaternion<K>;
using Quaterniond = Quaternion<double>;
using Quaternionf = Quaternion<float>;

// Helper function to compute the sum of two quaternions
template <typename K>
Quaternion<K> operator+(const Quaternion<K>& lhs, const Quaternion<K>& rhs) {
  return Quaternion<K>(lhs.coeffs() + rhs.coeffs());
}
// Helper function to compute the difference of two quaternions
template <typename K>
Quaternion<K> operator-(const Quaternion<K>& lhs, const Quaternion<K>& rhs) {
  return Quaternion<K>(lhs.coeffs() - rhs.coeffs());
}
// Unary - operator for a quaternion
template <typename K>
Quaternion<K> operator-(const Quaternion<K>& q) {
  return Quaternion<K>(-q.coeffs());
}

template <typename K>
using EigenImage = Eigen::Array<K, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
template <typename K>
using EigenImageMap = Eigen::Map<EigenImage<K>>;
template <typename K>
using EigenImageConstMap = Eigen::Map<const EigenImage<K>>;

// A base helper for constructing Views of data as eigen matrix maps.
template <typename K, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic, int MatrixOptions = 0,
          int MapOptions = 0, int OuterStride = 0, int InnerStride = 0>
using EigenMatrixView = Eigen::Map<Eigen::Matrix<K, Rows, Cols, MatrixOptions>, MapOptions,
                                   Eigen::Stride<OuterStride, InnerStride>>;

// A base helper for constructing const Views of data as eigen matrix maps.
template <typename K, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic, int MatrixOptions = 0,
          int MapOptions = 0, int OuterStride = 0, int InnerStride = 0>
using EigenMatrixConstView = Eigen::Map<const Eigen::Matrix<K, Rows, Cols, MatrixOptions>,
                                        MapOptions, Eigen::Stride<OuterStride, InnerStride>>;

// A base helper for constructing Views of data as eigen vector maps.
template <typename K, int Rows = Eigen::Dynamic, int MapOptions = 0,
          int OuterStride = 0, int InnerStride = 0>
using EigenVectorView = Eigen::Map<Eigen::Matrix<K, Rows, 1>, MapOptions,
                                   Eigen::Stride<OuterStride, InnerStride>>;

// A base helper for constructing const Views of data as eigen vector maps.
template <typename K, int Rows = Eigen::Dynamic,  int MapOptions = 0,
          int OuterStride = 0, int InnerStride = 0>
using EigenVectorConstView = Eigen::Map<const Eigen::Matrix<K, Rows, 1>, MapOptions,
                                        Eigen::Stride<OuterStride, InnerStride>>;

// A base helper for constructing Views of data as eigen row vector maps.
template <typename K, int Cols = Eigen::Dynamic, int MapOptions = 0,
          int OuterStride = 0, int InnerStride = 0>
using EigenRowVectorView = Eigen::Map<Eigen::Matrix<K, 1, Cols, Eigen::RowMajor>, MapOptions,
                                      Eigen::Stride<OuterStride, InnerStride>>;

// A base helper for constructing const Views of data as eigen row vector maps.
template <typename K, int Cols = Eigen::Dynamic, int MapOptions = 0,
          int OuterStride = 0, int InnerStride = 0>
using EigenRowVectorConstView = Eigen::Map<const Eigen::Matrix<K, 1, Cols, Eigen::RowMajor>,
                                           MapOptions, Eigen::Stride<OuterStride, InnerStride>>;

// Creates a Vector from an initializer list
template <typename K, size_t N>
Vector<K, N> MakeVector(const K (&elements)[N]) {
  Vector<K, N> result;
  for (size_t i = 0; i < N; i++) {
    result[i] = elements[i];
  }
  return result;
}

}  // namespace isaac
