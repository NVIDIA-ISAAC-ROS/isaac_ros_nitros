/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/geometry/line_segment.hpp"
#include "engine/gems/geometry/n_cuboid.hpp"
#include "engine/gems/geometry/n_sphere.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
using Json = nlohmann::json;

namespace serialization {

// Gets the value from the json for the given key.
template <typename T>
std::optional<T> TryGet(const Json& json);

// Sets a value in the json under the given key.
template <typename T>
void Set(Json& json, T value);

// Extracts the value for the given key from a JSON object. If the key does not exist in the JSON
// object, or if the type does not match the desired type nullopt will be returned.
template <typename T>
std::optional<T> TryGetFromMap(const Json& json, const std::string& key) {
  const auto it = json.find(key);
  if (it == json.end()) {
    return std::nullopt;
  } else {
    return TryGet<T>(*it);
  }
}

// Similar to `TryGetFromMap`, but will use the given default if the value could not be read
template <typename T>
T GetFromMapOrDefault(const Json& json, const std::string& key, T backup) {
  if (auto maybe = TryGetFromMap<T>(json, key)) {
    return *maybe;
  } else {
    return std::move(backup);
  }
}

// Given key_prefix, tries to get the angle both in radians and degrees, e.g.,
// if the key_prefix is "yaw", checks both "yaw_radians" and "yaw_degrees" values.
// If only one of these keys exist, returns the angle in radians.
template<class T, typename = std::enable_if_t<std::is_floating_point<T>::value>>
std::optional<T> TryGetAngleInRadiansFromMap(const Json& json, const std::string& key_prefix) {
  const std::string key_radians = key_prefix + "_radians";
  const std::string key_degrees = key_prefix + "_degrees";
  auto maybe_radians = TryGetFromMap<T>(json, key_radians);
  auto maybe_degrees = TryGetFromMap<T>(json, key_degrees);
  if (maybe_radians && maybe_degrees) {
    LOG_ERROR("Please set \"%s\" or \"%s\" only.", key_radians.c_str(), key_degrees.c_str());
    return std::nullopt;
  } else if (maybe_radians) {
    return *maybe_radians;
  } else if (maybe_degrees) {
    return DegToRad(*maybe_degrees);
  }
  // Angle is not set
  return std::nullopt;
}

//--------------------------------------------------------------------------------------------------

namespace json_formatter_details {

// A helper class used to format data for json.
// Synposis:
// static std::optional<T> TryGet(const Json::json& json);
// static void Set(const Json::json& json, T value);
template <typename T>
struct JsonSerializer {
  static std::optional<T> TryGet(const Json& json) {
    T x;
    try {
      from_json(json, x);
    } catch(...) {
      return std::nullopt;
    }
    return std::move(x);
  }
  static void Set(Json& json, T value) {
    to_json(json, value);
  }
};

// Support for getting JSON from JSON
template <>
struct JsonSerializer<Json> {
  static const Json& TryGet(const Json& json) {
    return json;
  }
  static void Set(Json& json, Json value) {
    json = std::move(value);
  }
};

// Parameter support for basic types: string, int, float, double, bool
#define ISAAC_JSON_FORMATTER_FLOAT_PRIMITIVE(TYPE) \
  template <> \
  struct JsonSerializer<TYPE> { \
    static std::optional<TYPE> TryGet(const Json& json) { \
      const double* double_ptr = json.get_ptr<const double*>(); \
      if (double_ptr != nullptr) { \
        return static_cast<TYPE>(*double_ptr); \
      } \
      const int64_t* int_ptr = json.get_ptr<const int64_t*>(); \
      if (int_ptr == nullptr) { \
        return std::nullopt; \
      } else { \
        return static_cast<TYPE>(*int_ptr); \
      } \
    } \
    static void Set(Json& json, TYPE value) { \
      json = static_cast<double>(value); \
    } \
  };
ISAAC_JSON_FORMATTER_FLOAT_PRIMITIVE(float);
ISAAC_JSON_FORMATTER_FLOAT_PRIMITIVE(double);
// Parameter support for basic types: string, int, float, double, bool
#define ISAAC_JSON_FORMATTER_PRIMITIVE(TYPE, JTYPE) \
  template <> \
  struct JsonSerializer<TYPE> { \
    static std::optional<TYPE> TryGet(const Json& json) { \
      const JTYPE* ptr = json.get_ptr<const JTYPE*>(); \
      if (ptr == nullptr) { \
        return std::nullopt; \
      } else { \
        return static_cast<TYPE>(*ptr); \
      } \
    } \
    static void Set(Json& json, TYPE value) { \
      json = static_cast<JTYPE>(value); \
    } \
  };
ISAAC_JSON_FORMATTER_PRIMITIVE(uint64_t, uint64_t);
ISAAC_JSON_FORMATTER_PRIMITIVE(int64_t, int64_t);
ISAAC_JSON_FORMATTER_PRIMITIVE(int, int64_t);
ISAAC_JSON_FORMATTER_PRIMITIVE(bool, bool);
ISAAC_JSON_FORMATTER_PRIMITIVE(std::string, std::string);

// Parameter support for compile-sized Vectors. It uses doubles to store elements in the json.
template <typename K, int N>
struct JsonSerializer<Vector<K, N>> {
  static std::optional<Vector<K, N>> TryGet(const Json& json) {
    if (!json.is_array()) {
      return std::nullopt;
    }
    const int json_size = static_cast<int>(json.size());
    if (N != Eigen::Dynamic && json_size != N) {
      return std::nullopt;
    }
    Vector<K, N> result(json_size);
    for (int i = 0; i < json_size; i++) {
      auto maybe = ::isaac::serialization::TryGet<K>(json[i]);
      if (!maybe) {
        return std::nullopt;
      }
      result[i] = *maybe;
    }
    return result;
  }
  static void Set(Json& json, const Vector<K, Eigen::Dynamic>& value) {
    json = Json(std::vector<K>(value.data(), value.data() + value.size()));
  }
  template<int M = N>
  static void Set(Json& json, const Vector<K, M>& value) {
    std::array<K, M> data;
    for (int i = 0; i < M; i++) {
      data[i] = value[i];
    }
    json = Json(data);
  }
};

// For std::vector
template <typename T>
struct JsonSerializer<std::vector<T>> {
  static std::optional<std::vector<T>> TryGet(const Json& json) {
    if (!json.is_array()) {
      return std::nullopt;
    }
    const size_t size = json.size();
    std::vector<T> result;
    result.reserve(size);
    for (size_t i = 0; i < size; i++) {
      auto maybe = ::isaac::serialization::TryGet<T>(json[i]);
      if (!maybe) {
        return std::nullopt;
      }
      result.push_back(*maybe);
    }
    return result;
  }
  static void Set(Json& json, const std::vector<T>& value) {
    json = Json::array();
    for (const auto& v : value) {
      Json vjson;
      ::isaac::serialization::Set(vjson, v);
      json.push_back(vjson);
    }
  }
};

// For std::array
template <typename T, size_t N>
struct JsonSerializer<std::array<T, N>> {
  static std::optional<std::array<T, N>> TryGet(const Json& json) {
    if (!json.is_array()) {
      return std::nullopt;
    }
    if (json.size() != N) {
      return std::nullopt;
    }
    std::array<T, N> result;
    for (size_t i = 0; i < N; i++) {
      auto maybe = ::isaac::serialization::TryGet<T>(json[i]);
      if (!maybe) {
        return std::nullopt;
      }
      result[i] = *maybe;
    }
    return result;
  }
  static void Set(Json& json, const std::array<T, N>& value) {
    json = Json::array();
    // TODO reserve
    for (const auto& v : value) {
      Json vjson;
      ::isaac::serialization::Set(vjson, v);
      json.push_back(vjson);
    }
  }
};

// For std::pair
template <typename T1, typename T2>
struct JsonSerializer<std::pair<T1, T2>> {
  static std::optional<std::pair<T1, T2>> TryGet(const Json& json) {
    if (!json.is_array()) {
      return std::nullopt;
    }
    if (json.size() != 2) {
      return std::nullopt;
    }
    auto maybe1 = ::isaac::serialization::TryGet<T1>(json[0]);
    if (!maybe1) {
      return std::nullopt;
    }
    auto maybe2 = ::isaac::serialization::TryGet<T2>(json[1]);
    if (!maybe2) {
      return std::nullopt;
    }
    return std::pair<T1, T2>{*maybe1, *maybe2};
  }
  static void Set(Json& json, const std::pair<T1, T2>& value) {
    Json value1;
    ::isaac::serialization::Set(value1, value.first);
    Json value2;
    ::isaac::serialization::Set(value2, value.second);
    json = Json::array({value1, value2});
  }
};

// For std::map<std::string, T>
// Supporting general std::map<T1, T2> would require array json, which we want to avoid for the time
// being
template <typename T>
struct JsonSerializer<std::map<std::string, T>> {
  static std::optional<std::map<std::string, T>> TryGet(const Json& json) {
    if (!json.is_object()) {
      return std::nullopt;
    }
    std::map<std::string, T> result;
    for (const auto& val : json.items()) {
      auto maybe = ::isaac::serialization::TryGet<T>(val.value());
      if (!maybe) {
        return std::nullopt;
      }
      result.emplace(val.key(), *maybe);
    }
    return result;
  }
  static void Set(Json& json, const std::map<std::string, T>& value) {
    json = Json::object();
    for (const auto& v : value) {
      json.emplace(v.first, v.second);
    }
  }
};

// Parameter support for SO3. It uses doubles to store elements in the json.
template <typename K>
struct JsonSerializer<SO3<K>> {
  // Tries to get pose in two ways. The first method is kept for backward compatibility.
  static std::optional<SO3<K>> TryGet(const Json& json) {
    // First method for backward compatibility.
    // Expected four elements are (w, x, y, z) values for the quaternion.
    // Example: [0.0, 0.0, 0.0, 1.0]
    if (json.is_array()) {
      auto maybe = JsonSerializer<Vector<K, 4>>::TryGet(json);
      if (!maybe) {
        return std::nullopt;
      }
      const auto& array = *maybe;
      return SO3<K>::FromQuaternion(Quaternion<K>{array[0], array[1], array[2], array[3]});
    }

    // Second method for convenience.
    // Rotation can set by one of:
    // { "axis": [x, y, z], "angle_radians": a }
    // { "axis": [x, y, z], "angle_degrees": a }
    // { "roll_radians": r, "pitch_radians": p, "yaw_radians": y }
    // { "roll_degrees": r, "pitch_degrees": p, "yaw_degrees": y }
    // { "qx": x, "qy": y, "qz": z, "qw": w }

    // We'll demand one and only one way to be used.
    size_t num_rotation_ways_set = 0;
    // If error message gets populated, this function will return null.
    // This variable will help us prioritize num_rotation_ways_set check when printing an error.
    std::string error_message;
    SO3<K> rotation = SO3<K>::Identity();

    // { "axis": [x, y, z], "angle_radians": a }
    // { "axis": [x, y, z], "angle_degrees": a }
    auto maybe_axis = TryGetFromMap<Vector3<K>>(json, "axis");
    auto maybe_angle = TryGetAngleInRadiansFromMap<K>(json, "angle");
    if (maybe_axis || maybe_angle) {
      num_rotation_ways_set++;
      if (maybe_angle && maybe_axis) {
        rotation = SO3<K>::FromAngleAxis(*maybe_angle, *maybe_axis);
      } else {
        error_message = "Both \"axis\" and \"angle\" need to be set.";
      }
    }

    // { "roll_radians": r, "pitch_radians": p, "yaw_radians": y }
    // { "roll_degrees": r, "pitch_degrees": p, "yaw_degrees": y }
    auto maybe_roll = TryGetAngleInRadiansFromMap<K>(json, "roll");
    auto maybe_pitch = TryGetAngleInRadiansFromMap<K>(json, "pitch");
    auto maybe_yaw = TryGetAngleInRadiansFromMap<K>(json, "yaw");
    if (maybe_roll || maybe_pitch || maybe_yaw) {
      num_rotation_ways_set++;
      if (!maybe_roll) maybe_roll = 0.0;
      if (!maybe_pitch) maybe_pitch = 0.0;
      if (!maybe_yaw) maybe_yaw = 0.0;
      rotation = SO3<K>::FromEulerAnglesRPY(*maybe_roll, *maybe_pitch, *maybe_yaw);
    }

    // { "qx": x, "qy": y, "qz": z, "qw": w }
    auto maybe_qx = TryGetFromMap<K>(json, "qx");
    auto maybe_qy = TryGetFromMap<K>(json, "qy");
    auto maybe_qz = TryGetFromMap<K>(json, "qz");
    auto maybe_qw = TryGetFromMap<K>(json, "qw");
    if (maybe_qx || maybe_qy || maybe_qz || maybe_qw) {
      num_rotation_ways_set++;
      if (maybe_qx && maybe_qy && maybe_qz && maybe_qw) {
        rotation =
            SO3<K>::FromQuaternion(Quaternion<K>{*maybe_qw, *maybe_qx, *maybe_qy, *maybe_qz});
      } else {
        error_message = "\"qx\", \"qy\", \"qz\" and \"qw\" all need to be set.";
      }
    }

    // check that only one way is used.
    if (num_rotation_ways_set != 1) {
      LOG_ERROR("Rotation can be one of:");
      LOG_ERROR("{ \"axis\": [x, y, z], \"angle\": a }");
      LOG_ERROR("{ \"roll\": r, \"pitch\": p, \"yaw\": y }");
      LOG_ERROR("{ \"qx\": x, \"qy\": y, \"qz\": z, \"qw\": w }");
      LOG_ERROR("Please set one and only one of them.");

      return std::nullopt;
    }

    // If we had an issue, print the error and return null
    if (!error_message.empty()) {
      LOG_ERROR(error_message.c_str());
      return std::nullopt;
    }

    return rotation;
  }
  static void Set(Json& json, const SO3<K>& value) {
    json = Json(std::array<K, 4>{{
                  value.quaternion().w(), value.quaternion().x(),
                  value.quaternion().y(), value.quaternion().z()
                }});
  }
};

// Parameter support for NSphere (where N is the dimension of the sphere)
// Input serialization formats
//   In order of priority TryGet() will look in "json" for:
//     (1) A std::pair for which the first component contains an array of N numbers which represent
//         the center coordinates of the NSphere and the second component contains a single scalar
//         value representing the radius of the NSphere.
//         Ex. Json json = {{1.0, 2.0}, 3.0};
//             represents a circle with center (1.0, 2.0) and radius 3.0
//     (2) A Json object named "center" containing an array of N numbers which represent the center
//         coordinates of the NSphere AND a Json object named "radius" containing a single scalar
//         value representing the radius of the NSphere
//         Ex. Json json;
//             json["center"] = {1.0, 2.0, 3.0};
//             json["radius"] = 4.0;
//             represents a sphere with center (1.0, 2.0, 3.0) and radius 4.0
//   If neither format is available, std::nullopt will be returned.
// Output serialization format:
//   A std::pair for which the first component contains an array of N numbers which represent the
//   center coordinates of the NSphere and the second component contains a single scalar value
//   representing the radius of the NSphere.
template <typename K, int N>
struct JsonSerializer<geometry::NSphere<K, N>> {
  using Sphere_t = geometry::NSphere<K, N>;
  using Vector_t  = typename geometry::NSphere<K, N>::Vector_t;
  using Scalar_t  = typename geometry::NSphere<K, N>::Scalar;
  static std::optional<Sphere_t> TryGet(const Json& json) {
    // Try to get NSphere from format: {center_vector, radius_scalar}
    auto maybe = JsonSerializer<std::pair<Vector_t, Scalar_t>>::TryGet(json);
    if (maybe) {
      return Sphere_t{maybe->first, maybe->second};
    }

    // Try to get NSphere from named attributes for center and radius
    auto maybe_center = TryGetFromMap<Vector_t>(json, "center");
    auto maybe_radius = TryGetFromMap<Scalar_t>(json, "radius");
    if (maybe_center && maybe_radius) {
      return Sphere_t{*maybe_center, *maybe_radius};
    }

    return std::nullopt;
  }
  static void Set(Json& json, const Sphere_t& value) {
    ::isaac::serialization::Set(json, std::pair<Vector_t, Scalar_t>{value.center, value.radius});
  }
};

// Parameter support for NCuboid (where N is the dimension of the cuboid)
// Input serialization formats
//   In order of priority TryGet() will look in "json" for:
//     (1) A std::array of length N, for which each component contains an the minimum and maximum
//         bound for the Nth dimension.
//         Ex. Json json = {{-1.0, 1.0}, {-2.0, 2.0}};
//             represents a rectangle with the x dimension spanning -1.0 to 1.0 and the y dimension
//             spanning -2.0 to 2.0.
//     (2) A Json object named "bounds" containing an array of length the N, for which each
//         component contains an the minimum and maximum bound for that dimension.
//         Ex. Json json;
//             json["bounds"] = {{-1.0, 1.0}, {-2.0, 2.0}};
//             represents a rectangle with the x dimension spanning -1.0 to 1.0 and the y dimension
//             spanning -2.0 to 2.0.
//     (3) A Json object named "corner_1" containing an array of length the N representing the
//         coordinates of one corner AND another Json object named "corner_2" containing an array of
//         length the N representing the coordinates of the opposite corner.
//         Ex. Json json;
//             json["corner_1"] = {-1.0, -2.0};
//             json["corner_2"] = { 1.0,  2.0};
//             represents a rectangle with the x dimension spanning -1.0 to 1.0 and the y dimension
//             spanning -2.0 to 2.0.
//     (4) A Json object named "center" containing an array of length the N representing the
//         coordinates of the center of the NCuboid AND another Json object named "dimensions"
//         containing an array of length the N representing the length of each dimension.
//         Ex. Json json;
//             json["center"] = {0.0, 0.0};
//             json["dimension"] = {2.0, 4.0};
//             represents a rectangle with the x dimension spanning -1.0 to 1.0 and the y dimension
//             spanning -2.0 to 2.0.
//   If none of these formats are available, std::nullopt will be returned.
// Output serialization format:
//   A std::array of length N, for which each component contains an the minimum and maximum
//   bound for the Nth dimension.
template <typename K, int N>
struct JsonSerializer<geometry::NCuboid<K, N>> {
  using Cube_t = geometry::NCuboid<K, N>;
  using Vector_t  = typename geometry::NCuboid<K, N>::Vector_t;
  using Vector2K  = Vector<K, 2>;
  using ArrayNVector2K = std::array<Vector2K, N>;
  static std::optional<Cube_t> TryGet(const Json& json) {
    // Try to get NCuboid from bounding cuboid format:
    // {{min_dim_0, max_dim_0}, {min_dim_1, max_dim_1}, ... {min_dim_N, max_dim_N}}
    auto maybe = JsonSerializer<std::array<Vector2K, N>>::TryGet(json);
    if (maybe) {
      return Cube_t::FromBoundingCuboid(*maybe);
    }

    // Try to get NCuboid from named bounds
    auto maybe_bounds = TryGetFromMap<ArrayNVector2K>(json, "bounds");
    if (maybe_bounds) {
      return Cube_t::FromBoundingCuboid(*maybe_bounds);
    }

    // Try to get NCuboid from named opposite corners
    auto maybe_corner_1 = TryGetFromMap<Vector_t>(json, "corner_1");
    auto maybe_corner_2 = TryGetFromMap<Vector_t>(json, "corner_2");
    if (maybe_corner_1 && maybe_corner_2) {
      return Cube_t::FromOppositeCorners(*maybe_corner_1, *maybe_corner_2);
    }

    // Try to get NCuboid from named center and dimensions
    auto maybe_center = TryGetFromMap<Vector_t>(json, "center");
    auto maybe_dimensions = TryGetFromMap<Vector_t>(json, "dimensions");
    if (maybe_center && maybe_dimensions) {
      return Cube_t::FromSizes(*maybe_center, *maybe_dimensions);
    }

    return std::nullopt;
  }
  static void Set(Json& json, const Cube_t& value) {
    ArrayNVector2K bounds;
    for (size_t dim = 0; dim < N; dim++) {
      bounds[dim] = Vector2K(value.min()[dim], value.max()[dim]);
    }
    ::isaac::serialization::Set(json, ArrayNVector2K{bounds});
  }
};

// Parameter support for LineSegment
// Input serialization formats
//   In order of priority TryGet() will look in "json" for:
//     (1) A std::pair for which the first component contains an array of N numbers which represent
//         the coordinates of the first endpoint and the second component contains an array of N
//         numbers which represent the coordinates of the second endpoint.
//         Ex. Json json = {{1.0, 2.0}, {3.0, 4.0}};
//             represents a 2D line segment spanning from (1.0, 2.0) to (3.0, 4.0).
//     (2) A Json object named "point_1" containing an array of N numbers which represent the
//         coordinates of the first endpoint AND a Json object named "point_2" containing an array
//         of N numbers which represent the coordinates of the second endpoint.
//         Ex. Json json;
//             json["point_1"] = {1.0, 2.0};
//             json["point_2"] = {3.0, 4.0};
//             represents a 2D line segment spanning from (1.0, 2.0) to (3.0, 4.0).
//   If neither format is available, std::nullopt will be returned.
// Output serialization format:
//   A std::pair for which the first component contains an array of N numbers which represent the
//   coordinates of the first endpoint and the second component contains an array of N numbers which
//   represent the coordinates of the second endpoint.
template <typename K, int N>
struct JsonSerializer<geometry::LineSegment<K, N>> {
  using LineSegment_t = geometry::LineSegment<K, N>;
  using Vector_t  = typename geometry::LineSegment<K, N>::Vector_t;
  static std::optional<LineSegment_t> TryGet(const Json& json) {
    // Try to get LineSegment from format: {point_1_vector, point_2_vector}
    auto maybe = JsonSerializer<std::pair<Vector_t, Vector_t>>::TryGet(json);
    if (maybe) {
      return LineSegment_t{maybe->first, maybe->second};
    }

    // Try to get LineSegment from named attributes for point_1 and point_2
    auto maybe_point_1 = TryGetFromMap<Vector_t>(json, "point_1");
    auto maybe_point_2 = TryGetFromMap<Vector_t>(json, "point_2");
    if (maybe_point_1 && maybe_point_2) {
      return LineSegment_t{*maybe_point_1, *maybe_point_2};
    }

    return std::nullopt;
  }
  static void Set(Json& json, const LineSegment_t& value) {
    ::isaac::serialization::Set(json, std::pair<Vector_t, Vector_t>{value.a(), value.b()});
  }
};

// Parameter support for Pose3. It uses doubles to store elements in the json.
template <typename K>
struct JsonSerializer<Pose3<K>> {
  // Tries to get pose in two ways. The first method is kept for backward compatibility.
  static std::optional<Pose3<K>> TryGet(const Json& json) {
    // First method for backward compatibility.
    // First four elements are (w, x, y, z) values for the quaternion.
    // Last three elements are (x, y, z) locations.
    // Example: [0.0, 0.0, 0.0, 1.0, 25.0, 20.0, 0.0]
    if (json.is_array()) {
      auto maybe = JsonSerializer<Vector<K, 7>>::TryGet(json);
      if (!maybe) {
        return std::nullopt;
      }
      const auto& array = *maybe;
      return Pose3<K>{SO3<K>::FromQuaternion(Quaternion<K>{array[0], array[1], array[2], array[3]}),
                      Vector3<K>{array[4], array[5], array[6]}};
    }

    // Second method for convenience.
    // User can supply a translation. If not supplied, zero translation will be used.
    // User can also supply a rotation. If not supplied, identity matrix (zero rotation) will be
    // used.
    // The expected format is:
    // {
    //    "translation": [x, y, z],
    //    "rotation": R
    // }
    // where acceptable formats for R are listed in SO3 serialization.
    Vector3<K> translation =
        GetFromMapOrDefault<Vector3<K>>(json, "translation", Vector3<K>::Zero());
    SO3<K> rotation = GetFromMapOrDefault<SO3<K>>(json, "rotation", SO3<K>::Identity());
    return Pose3<K>{rotation, translation};
  }
  static void Set(Json& json, const Pose3<K>& value) {
    json = Json(std::array<K, 7> {{
                  value.rotation.quaternion().w(), value.rotation.quaternion().x(),
                  value.rotation.quaternion().y(), value.rotation.quaternion().z(),
                  value.translation.x(), value.translation.y(), value.translation.z()
                }});
  }
};

// Parameter support for Pose2. It uses doubles to store elements in the json.
template <typename K>
struct JsonSerializer<Pose2<K>> {
  static std::optional<Pose2<K>> TryGet(const Json& json) {
    auto maybe = JsonSerializer<Vector<K, 3>>::TryGet(json);
    if (!maybe) {
      return std::nullopt;
    }
    const auto& array = *maybe;
    return Pose2<K>{
      SO2<K>::FromAngle(array[0]),
      Vector2<K>{array[1], array[2]}
    };
  }
  static void Set(Json& json, const Pose2<K>& value) {
    json = Json(std::array<K, 3>{{
                  value.rotation.angle(), value.translation.x(), value.translation.y()
                }});
  }
};

// Helper function to validate json color inputs for Pixel3ub and Pixel4ub
inline bool ValidateColorInput(const std::vector<int>& color_array) {
  if (!(color_array.size() == 3 || color_array.size() == 4)) {
    LOG_WARNING("Color must have three (RGB) or four (RGBA) components");
    return false;
  }
  for (int component : color_array) {
    if (!(component >= 0 && component <= 255)) {
       LOG_WARNING("Color component must be in range 0-255");
       return false;
     }
  }
  return true;
}

// Parameter support for Pixel3ub. It uses uint8_t to store elements in the json.
template <>
struct JsonSerializer<Pixel3ub> {
  static std::optional<Pixel3ub> TryGet(const Json& json) {
    auto maybe = JsonSerializer<std::vector<int>>::TryGet(json);
    if (!maybe) {
      return std::nullopt;
    }
    const auto& array = *maybe;
    if (ValidateColorInput(array)) {
      return Pixel3ub{
        static_cast<uint8_t>(array[0]),
        static_cast<uint8_t>(array[1]),
        static_cast<uint8_t>(array[2]),
      };
    } else {
      LOG_WARNING("Invalid color input");
      return std::nullopt;
    }
  }
  static void Set(Json& json, const Pixel3ub& value) {
    json = Json(std::array<uint8_t, 3>{{value[0], value[1], value[2]}});
  }
};

// Parameter support for Pixel4ub. It uses uint8_t to store elements in the json.
template <>
struct JsonSerializer<Pixel4ub> {
  static std::optional<Pixel4ub> TryGet(const Json& json) {
    auto maybe = JsonSerializer<std::vector<int>>::TryGet(json);
    if (!maybe) {
      return std::nullopt;
    }
    auto& array = *maybe;
    if (ValidateColorInput(array)) {
      // Add opaque alpha channel if not supplied in Json file
      if (array.size() == 3) { array.push_back(255); }

      return Pixel4ub{
        static_cast<uint8_t>(array[0]),
        static_cast<uint8_t>(array[1]),
        static_cast<uint8_t>(array[2]),
        static_cast<uint8_t>(array[3])
      };
    } else {
      LOG_WARNING("Invalid color input");
      return std::nullopt;
    }
  }
  static void Set(Json& json, const Pixel4ub& value) {
    json = Json(std::array<uint8_t, 4>{{value[0], value[1], value[2], value[3]}});
  }
};

}  // namespace json_formatter_details

template <typename T>
std::optional<T> TryGet(const Json& json) {
  return json_formatter_details::JsonSerializer<T>::TryGet(json);
}

template <typename T>
void Set(Json& json, T value) {
  json_formatter_details::JsonSerializer<T>::Set(json, std::move(value));
}

}  // namespace serialization
}  // namespace isaac

namespace Eigen {

// Conversion from Eigen types like Vector2d to Json
template <typename T>
void to_json(nlohmann::json& json, const T& value) {
  ::isaac::serialization::Set(json, value);
}

// Conversion from Json to Eigen types like Vector2d
template <typename T>
void from_json(const nlohmann::json& json, T& value) {
  if (auto maybe = ::isaac::serialization::TryGet<T>(json)) {
    value = *maybe;
  }
}

}  // namespace Eigen
