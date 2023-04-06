/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/line_segment.hpp"
#include "engine/gems/geometry/line_utils.hpp"
#include "engine/gems/geometry/n_cuboid.hpp"
#include "engine/gems/geometry/n_sphere.hpp"

namespace isaac {
namespace geometry {

// A polygon in 2D, the last point and first point are connected
template <typename K>
struct Polygon2 {
  // Type of a point.
  using Vector_t = Vector2<K>;
  using Scalar = K;

  std::vector<Vector_t> points;

  // Returns whether a point is inside the polygon
  // Note that this method is not accurate if the point lies too close to one edge, Especially if
  // the point is really close to the edge or of a vertex, it might return true.
  // This function used the strategy of counting the intersection between the segments of the
  // polygon and a virtual segment from the `pt` to a point outside of the polygon. While fast, this
  // method does not work well when the virtual segment is too close to one of the vertices. In this
  // case it fallbacks to the slower implementation below.
  bool isInside(const Vector_t& pt) const {
    if (points.empty()) return false;
    // Select a point outside of the polygon (max(x) + 1, max(y) + 2).
    // Picking a different constant for x and y helps reduce the chance the point is aligned with
    // one vertex.
    Vector_t max = points.front();
    for (const auto& point : points) {
      max.x() = std::max(max.x(), point.x() + K(1));
      max.y() = std::max(max.y(), point.y() + K(2));
    }
    // If the maximum value of x+1 is less or equal to our point, then we are obviously outside.
    if (max.x() <= pt.x()) return false;
    if (max.y() <= pt.y()) return false;
    const LineSegment<K, 2> seg(pt, max);
    // Count how many segment it intersects, if it's an odd number then the point is inside or on
    // one edge.
    int counter = 0;
    for (size_t ix = 0; ix < points.size(); ix++) {
      const LineSegment<K, 2> edge(points[ix], points[(ix + 1) % points.size()]);
      K lambda;
      if (AreLinesIntersecting(seg, edge, nullptr, &lambda)) {
        counter++;
        // If our lines are too close, we fallback to the slow but accurate method.
        if (IsAlmostZero(lambda) || IsAlmostOne(lambda)) {
          return slowIsInside(pt);
        }
      }
    }
    return counter % 2 == 1;
  }

  // Returns whether a point is inside the polygon
  // This function is a bit slow and should not be called in critical path too often.
  // This function used the strategy to measure the sum of the field of view the segment are
  // observed. It was the most accurate solution, however it requires using trigonometric functions,
  // which are quite slow.
  bool slowIsInside(const Vector_t& pt) const {
    if (points.empty()) return false;
    K sum_angle = K(0);
    Vector_t delta = points.back() - pt;
    // If delta == 0, it means pt is the same as one of the vertex.
    if (IsAlmostZero(delta.squaredNorm())) return true;
    K last_angle = std::atan2(delta.y(), delta.x());
    for (size_t ix = 0; ix < points.size(); ix++) {
      delta = points[ix] - pt;
      // If delta == 0, it means pt is the same as one of the vertex.
      if (IsAlmostZero(delta.squaredNorm())) return true;
      const K angle = std::atan2(delta.y(), delta.x());
      const K delta_angle = DeltaAngle(angle, last_angle);
      // If delta == +/- pi, it means pt is on one of the segment.
      if (IsAlmostEqualRelative(std::abs(delta_angle), Pi<K>)) {
        return true;
      }
      sum_angle += delta_angle;
      last_angle = angle;
    }
    return std::abs(sum_angle) > Pi<K>;
  }

  // Returns the distance of a point from the polygon edges
  K distance(const Vector_t& pt, Vector_t* grad = nullptr) const {
    K squared_dist = std::numeric_limits<K>::max();
    for (size_t ix = 0; ix < points.size(); ix++) {
      const LineSegment<K, 2> edge(points[ix], points[(ix + 1) % points.size()]);
      const Vector_t closest = ClosestPointToLine(edge, pt);
      const K dist = (pt - closest).squaredNorm();
      if (dist < squared_dist) {
        squared_dist = dist;
        if (grad) *grad = pt - closest;
      }
    }
    if (grad) grad->normalize();
    return std::sqrt(squared_dist);
  }

  // Returns the signed distance of a point from the polygon (negative means inside the polygon)
  K signedDistance(const Vector_t& pt, Vector_t* grad = nullptr) const {
    const K dist = distance(pt, grad);
    // If the point is inside the polygon, we need to revert the direction of the gradient.
    if (isInside(pt)) {
      if (grad) *grad = -(*grad);
      return -dist;
    }
    return dist;
  }

  // Returns the signed area of the polygon
  // https://mathworld.wolfram.com/PolygonArea.html
  K signedArea() const {
    K area = K(0);
    for (size_t curr = 0, prev = points.size()-1; curr < points.size(); curr++) {
      area += CrossProduct(points[prev], points[curr]);
      prev = curr;
    }
    return area / K(2);
  }

  // Returns the centroid of the polygon
  // https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
  Vector_t centroid() const {
    if (points.size() < 3) {
      if (points.empty()) return Vector_t::Zero();
      return (points.front() + points.back()) / K(2);
    }
    Vector_t centroid = Vector_t::Zero();
    for (size_t curr = 0, prev = points.size()-1; curr < points.size(); curr++) {
      const K cross_product = CrossProduct(points[prev], points[curr]);
      centroid += (points[prev] + points[curr]) * cross_product;
      prev = curr;
    }
    return centroid / (K(6) * signedArea());
  }

  // Returns the minimum circle that contains the polygon.
  Circle<K> boundingCircle() const {
    std::vector<size_t> indices(points.size());
    std::vector<size_t> border;
    for (size_t idx = 0; idx < points.size(); idx++) {
      indices[idx] = idx;
    }
    // Random order to improve performance
    std::random_shuffle(indices.begin(), indices.end());
    // Compute the center when the number of vertices is less or equal to 3
    auto trivialCenter = [&](const std::vector<size_t>& indices) -> Circle<K> {
      switch (indices.size()) {
        // Edge case for empty polygon, there is no real good answer, but any circle of radius 0
        // is technically a good answer.
        case 0: return {Vector_t::Zero(), K(0)};
        // If the polygon has a single point, then it's the center of the circle of radius 0.
        case 1: return {points[indices[0]], K(0)};
        // For two points, the center is the center of the segment, and the radius can be computed
        // using one of the extremity.
        case 2: {
          const Vector_t delta = (points[indices[1]] - points[indices[0]]) / K(2);
          return {points[indices[0]] + delta, delta.norm()};
        }
        // There is a formula for a triangle:
        // https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_2
        case 3: {
          const Vector_t& A = points[indices[0]];
          const Vector_t& B = points[indices[1]] - A;
          const Vector_t& C = points[indices[2]] - A;
          const K b_squared_norm = B.squaredNorm();
          const K c_squared_norm = C.squaredNorm();
          const K d = K(2) * CrossProduct(B, C);
          const Vector_t center = Vector_t(
              b_squared_norm * C.y() - c_squared_norm * B.y(),
              c_squared_norm * B.x() - b_squared_norm * C.x()) / d;
          return {A + center, center.norm()};
        }
        default:
          PANIC("Invalid size: %zd", indices.size());
      }
      return {Vector_t::Zero(), K(0)};
    };
    // Implementation of the welzl algorithm:
    // https://en.wikipedia.org/wiki/Smallest-circle_problem#Welzl's_algorithm
    auto welzl = [&](auto welzl, std::vector<size_t>& indices, std::vector<size_t>& border) {
      if (indices.empty() || border.size() == 3) return trivialCenter(border);
      size_t p = indices.back();
      indices.pop_back();
      auto circle = welzl(welzl, indices, border);
      if (circle.isInside(points[p])) {
        indices.push_back(p);
        return circle;
      }
      border.push_back(p);
      circle = welzl(welzl, indices, border);
      border.pop_back();
      indices.push_back(p);
      return circle;
    };
    return welzl(welzl, indices, border);
  }

  // Returns whether both polygons intersect. If one polygon is fully inside the other, then the
  // function returns false.
  bool intersect(const Polygon2& polygon) const {
    return intersectImpl<1>(polygon.points);
  }

  // Returns whether this polygon intersect with the polyline.
  bool intersect(const Polyline<K, 2>& polyline) const {
    return intersectImpl<0>(polyline);
  }

  // Returns the distance between two polygons.
  // If one polygon is stricly included in the other, the distance will be negative.
  K distance(const Polygon2& polygon) const {
    const K squared_dist = squaredDistanceImpl<1>(polygon.points);
    if (squared_dist == K(0)) return K(0);
    const auto bounding_box_a = getBoundingBox();
    const auto bounding_box_b = polygon.getBoundingBox();
    // If the bounding box A encapsulates B, then B might be inside A
    if (bounding_box_a.min().x() < bounding_box_b.min().x() &&
        bounding_box_b.max().x() < bounding_box_a.max().x() &&
        bounding_box_a.min().y() < bounding_box_b.min().y() &&
        bounding_box_b.max().y() < bounding_box_a.max().y()) {
      return isInside(polygon.points.front()) ? -std::sqrt(squared_dist) : std::sqrt(squared_dist);
    }

    // If the bounding box B encapsulates A, then A might be inside B
    if (bounding_box_b.min().x() < bounding_box_a.min().x() &&
        bounding_box_a.max().x() < bounding_box_b.max().x() &&
        bounding_box_b.min().y() < bounding_box_a.min().y() &&
        bounding_box_a.max().y() < bounding_box_b.max().y()) {
      return polygon.isInside(points.front()) ? -std::sqrt(squared_dist) : std::sqrt(squared_dist);
    }

    return std::sqrt(squared_dist);
  }

  // Returns the distance between this polygon and a polyline.
  // If the polyline is stricly included in this polygon, the distance will be negative.
  K distance(const Polyline<K, 2>& polyline) const {
    const K squared_dist = squaredDistanceImpl<0>(polyline);
    if (squared_dist == K(0)) return K(0);
    return isInside(polyline.front()) ? -std::sqrt(squared_dist) : std::sqrt(squared_dist);
  }

  // Returns the bounding box, aligned with the axis, that contain this polygon.
  Rectangle<K> getBoundingBox() const {
    if (points.empty()) {
      return Rectangle<K>::FromOppositeCorners(Vector2<K>::Zero(), Vector2<K>::Zero());
    }
    Vector2<K> min = points.front();
    Vector2<K> max = points.front();
    for (const auto& pt : points) {
      min = min.cwiseMin(pt);
      max = max.cwiseMax(pt);
    }
    return Rectangle<K>::FromOppositeCorners(min, max);
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  Polygon2<S> cast() const {
    std::vector<Vector2<S>> pts;
    pts.reserve(points.size());
    for (const auto& pt : points) {
      pts.emplace_back(pt.template cast<S>());
    }
    return Polygon2<S>{std::move(pts)};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const Polygon2& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }

 private:
  // Implementation of the squaredDistance using a polyline.
  // If loop is 1, then we close the polyline (making it a polygon).
  template<int Loop>
  K squaredDistanceImpl(const Polyline<K, 2>& polyline) const {
    static_assert(Loop >= 0 && Loop <= 1, "Loop must be 0 or 1");
    // Check for an intersection
    if (intersectImpl<Loop>(polyline)) return K(0);
    // Find the closest point of either polygon to the other one.
    K squared_dist = std::numeric_limits<K>::max();
    // For all the points of this polygon, we check the distance to all the segments of the other
    // polygon.
    for (size_t ax = 0; ax < points.size(); ax++) {
      const Vector2<K>& a = points[ax];
      const Vector2<K>& b = points[(ax + 1) % points.size()];
      for (size_t bx = 0; bx < polyline.size(); bx++) {
        squared_dist = std::min(squared_dist, DistanceSquaredPointToSegment(a, b, polyline[bx]));
      }
    }
    // For all the points of the other polygon, we check the distance to all the segments of this
    // polygon.
    for (size_t ax = 1; ax < polyline.size() + Loop; ax++) {
      const Vector2<K>& a = polyline[ax - 1];
      const Vector2<K>& b = polyline[ax % polyline.size()];
      for (size_t bx = 0; bx < points.size(); bx++) {
        squared_dist = std::min(squared_dist, DistanceSquaredPointToSegment(a, b, points[bx]));
      }
    }
    return squared_dist;
  }

  // Implementation of the intersect function with a polyline.
  // If loop is 1, then we close the polyline (making it a polygon).
  template<int Loop>
  K intersectImpl(const Polyline<K, 2>& polyline) const {
    static_assert(Loop >= 0 && Loop <= 1, "Loop must be 0 or 1");
    const auto bounding_box_a = getBoundingBox();
    const auto bounding_box_b = Polygon2{polyline}.getBoundingBox();
    if (bounding_box_a.min().x() > bounding_box_b.max().x() ||
        bounding_box_b.min().x() > bounding_box_a.max().x() ||
        bounding_box_a.min().y() > bounding_box_b.max().y() ||
        bounding_box_b.min().y() > bounding_box_a.max().y()) {
      return false;
    }
    for (size_t ax = 0; ax < points.size(); ax++) {
      const LineSegment<K, 2> a(points[ax], points[(ax + 1) % points.size()]);
      for (size_t bx = 1; bx < polyline.size() + Loop; bx++) {
        const LineSegment<K, 2> b(polyline[bx - 1], polyline[bx % polyline.size()]);
        if (AreLinesIntersecting(a, b)) return true;
      }
    }
    return false;
  }
};

// A polygon in 3D, the last point and first point are connected
template <typename K>
struct Polygon3 {
  // Type of a point.
  using Vector_t = Vector3<K>;
  using Scalar = K;

  std::vector<Vector_t> points;
};

using Polygon2D = Polygon2<double>;
using Polygon2F = Polygon2<float>;
using Polygon3D = Polygon3<double>;
using Polygon3F = Polygon3<float>;

}  // namespace geometry
}  // namespace isaac
