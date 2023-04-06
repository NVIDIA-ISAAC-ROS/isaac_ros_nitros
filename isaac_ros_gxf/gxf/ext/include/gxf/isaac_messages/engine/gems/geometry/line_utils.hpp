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
#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/geometry/polyline.hpp"

namespace isaac {
namespace geometry {

// Returns the closest point of a segment (ab) to a point `p`.
// NOTE: It can be used with Line/HalfLine/Segment.
template<typename Line>
typename Line::Vector_t ClosestPointToLine(const Line& line, const typename Line::Vector_t& p);

// Returns the squared distance between a point and a Line/HalfLine/Segment.
// Avoid any sqrt by using the pytagorian theorem.
template<typename Line>
typename Line::Scalar SquaredDistancePointToLine(const Line& line,
                                                 const typename Line::Vector_t& p);

// Returns the distance between a point and a Line/HalfLine/Segment.
template<typename Line>
typename Line::Scalar DistancePointToLine(const Line& line, const typename Line::Vector_t& p);

// Returns whether two Line/HalfLine/Segment are intersecting.
// In addition if the line intersect lambda_{A/B} would be filled such as:
// A.origin() + lambda_a A.direction() == B.origin() + lambda_b B.direction()
template<typename LineT1, typename LineT2>
bool AreLinesIntersecting(const LineT1& line_a, const LineT2& line_b,
                          typename LineT1::Scalar* lambda_a = nullptr,
                          typename LineT1::Scalar* lambda_b = nullptr);

// Returns the closest point of two lines. If the lines are intersecting, the intersection will be
// returned, otherwise (i.e. when the lines are parallel), any point of any line would work. Note
// that if a line segment is passed this function only looks at the underlying line and ignores the
// start and end of the segment.
template <typename LineT1, typename LineT2>
Vector2<typename LineT1::Scalar> Intersection(const LineT1& line_a, const LineT2& line_b);

// Returns the closest point of a segment [ab] to a point `p`.
template<typename K, int N>
Vector<K, N> ClosestPointToSegment(const Vector<K, N>& a, const Vector<K, N>& b,
                                   const Vector<K, N>& p);

// Returns the closest point of a polyline [ab...yz] to a point `p`.
// If `segment_index` is provided, it will contain the index of the segment the closest to the
// point.
template<typename K, int N>
Vector<K, N> ClosestPointToPolyline(const Polyline<K, N>& polyline, const Vector<K, N>& p,
                                    size_t* segment_index = nullptr);

// Returns the squared distance of a point `p` to a segment [ab].
template<typename K, int N>
K DistanceSquaredPointToSegment(const Vector<K, N>& a, const Vector<K, N>& b,
                                const Vector<K, N>& p);

// Returns the distance of a point `p` to a segment [ab].
template<typename K, int N>
K DistancePointToSegment(const Vector<K, N>& a, const Vector<K, N>& b, const Vector<K, N>& p);

// -------------------------------------------------------------------------------------------------

template<typename Line>
typename Line::Vector_t ClosestPointToLine(const Line& line, const typename Line::Vector_t& p) {
  const typename Line::Vector_t& ray = line.direction();
  const typename Line::Vector_t ap = p - line.origin();
  const typename Line::Scalar l = line.clamp(ray.dot(ap) / ray.squaredNorm());
  return line.origin() + l * ray;
}

template<typename Line>
typename Line::Scalar SquaredDistancePointToLine(const Line& line,
                                                 const typename Line::Vector_t& p) {
  using K = typename Line::Scalar;
  const typename Line::Vector_t& ray = line.direction();
  const typename Line::Vector_t ap = p - line.origin();
  const K ray_sq_norm = ray.squaredNorm();
  // Check this is not an empty segment, in which case the distance is the distance to one of the
  // extremities.
  if (IsAlmostZero(ray_sq_norm)) return ap.squaredNorm();
  const K dot = ray.dot(ap);
  const K l = dot / ray_sq_norm;
  const K cl = line.clamp(l);
  const K cl2 = cl * cl * ray_sq_norm;
  return ap.squaredNorm() + cl2 - K(2) * l * cl * ray_sq_norm;
}

template<typename Line>
typename Line::Scalar DistancePointToLine(const Line& line, const typename Line::Vector_t& p) {
  return std::sqrt(SquaredDistancePointToLine(line, p));
}

// P(x, y) is on a line iff (P-A)xR_a = 0
// We are looking for lambda such as (B + lambda R_b - A) x R_a = 0
// lambda = (A-B) x R_a / (R_b x R_a)
// x denotes the cross product here.
template<typename LineT1, typename LineT2>
bool AreLinesIntersecting(const LineT1& line_a, const LineT2& line_b,
                          typename LineT1::Scalar* lambda_a,
                          typename LineT1::Scalar* lambda_b) {
  static_assert(LineT1::kDimension == 2 && LineT2::kDimension == 2,
                "AreLinesIntersecting only works in 2D");
  static_assert(std::is_same<typename LineT1::Scalar, typename LineT2::Scalar>::value,
                "Type mismatch");
  using K = typename LineT1::Scalar;
  K lA, lB;
  if (lambda_a == nullptr) lambda_a = &lA;
  if (lambda_b == nullptr) lambda_b = &lB;
  const Vector2<K>& R_a = line_a.direction();
  const Vector2<K>& R_b = line_b.direction();
  const K ray_cross = CrossProduct(R_b, R_a);
  // The two lines are parallel, they could still intersect in one or an infinity of points.
  if (IsAlmostZero(ray_cross)) {
    const Vector2<K> pt = ClosestPointToLine(line_a, line_b.origin());
    constexpr K kEpsilonThreshold = MachineEpsilon<K> * 1000.0;
    if (SquaredDistancePointToLine(line_b, pt) >= kEpsilonThreshold) return false;
    const K R_a_norm = line_a.direction().squaredNorm();
    const K R_b_norm = line_b.direction().squaredNorm();
    *lambda_a = IsAlmostZero(R_a_norm) ? K(0) : (pt - line_a.origin()).dot(R_a) / R_a_norm;
    *lambda_b = IsAlmostZero(R_b_norm) ? K(0) : (pt - line_b.origin()).dot(R_b) / R_b_norm;
  } else {
    const Vector2<K> BA = line_a.origin() - line_b.origin();
    *lambda_b = CrossProduct(BA, R_a) / ray_cross;
    *lambda_a = CrossProduct(BA, R_b) / ray_cross;
  }
  // Checks that both lambda is within the range of the line.
  return line_a.clamp(*lambda_a) == *lambda_a && line_b.clamp(*lambda_b) == *lambda_b;
}

template<typename LineT1, typename LineT2>
Vector2<typename LineT1::Scalar> Intersection(const LineT1& line_a, const LineT2& line_b) {
  static_assert(std::is_same<typename LineT1::Scalar, typename LineT2::Scalar>(),
                "both line need to have the same type");
  using K = typename LineT1::Scalar;
  // Line equation is given by:
  // line.direction.tangent.dot(x, y) = line.direction.tangent.dot(line.origin)
  // We have a set of two equations, with two unknowns:
  // ( d1_y  -d1_x)   (x)   (a)
  // (            ) * ( ) = ( )
  // (-d2_y   d2_x)   (y)   (b)
  const Vector2<K> t1(line_a.direction().y(), -line_a.direction().x());
  const Vector2<K> t2(-line_b.direction().y(), line_b.direction().x());
  const K det = t1.dot(line_b.direction());
  // In this case the line are parallel.
  if (IsAlmostZero(det)) return line_a.origin();

  Matrix2<K> matrix_inverse;
  matrix_inverse << t2.y(), -t1.y(), -t2.x(), t1.x();
  const Vector2<K> result(line_a.origin().dot(t1), line_b.origin().dot(t2));
  return matrix_inverse * result / det;
}

template<typename K, int N>
Vector<K, N> ClosestPointToSegment(const Vector<K, N>& a, const Vector<K, N>& b,
                                   const Vector<K, N>& p) {
  const Vector<K, N> ab = b - a;
  const Vector<K, N> ap = p - a;
  const K l = ab.dot(ap);
  if (l <= K(0)) {
    return a;
  }
  const K ab_n2 = ab.squaredNorm();
  if (l >= ab_n2) {
    return b;
  }
  return a + (l / ab_n2) * ab;
}

template<typename K, int N>
Vector<K, N> ClosestPointToPolyline(const Polyline<K, N>& polyline, const Vector<K, N>& p,
                                    size_t* segment_index) {
  ASSERT(polyline.size() > 0, "Polyline is empty");
  if (segment_index) *segment_index = 0;
  K best_dist = (p - polyline.front()).squaredNorm();
  Vector<K, N> ret = polyline.front();
  for (size_t i = 1; i < polyline.size(); i++) {
    const auto vec = ClosestPointToSegment(polyline[i - 1], polyline[i], p);
    const K dist = (vec - p).squaredNorm();
    if (dist < best_dist) {
      best_dist = dist;
      ret = vec;
      if (segment_index) *segment_index = i-1;
    }
  }
  return ret;
}

template<typename K, int N>
K DistanceSquaredPointToSegment(const Vector<K, N>& a, const Vector<K, N>& b,
                                const Vector<K, N>& p) {
  const Vector<K, N> ab = b - a;
  const Vector<K, N> ap = p - a;
  const K l = ab.dot(ap);
  if (l <= K(0)) {
    return ap.squaredNorm();
  }
  const K ab_n2 = ab.squaredNorm();
  if (l >= ab_n2) {
    return (b - p).squaredNorm();
  }
  return ap.squaredNorm() - l * l / ab_n2;
}

template<typename K, int N>
K DistancePointToSegment(const Vector<K, N>& a, const Vector<K, N>& b, const Vector<K, N>& p) {
  return std::sqrt(DistanceSquaredPointToSegment(a, b, p));
}

}  // namespace geometry
}  // namespace isaac
