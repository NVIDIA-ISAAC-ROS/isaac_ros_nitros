// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

#pragma once

#include <memory>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Obstacles represent 3d geometries that can be added to `cumotion::World`.
//!
//! See `cumotion/world.h` for usage.
class CUMO_EXPORT Obstacle {
 public:
  //! Indicates what geometric primitive the obstacle represents.
  //!
  //! Each `Obstacle::Type` has one or more attributes that can be set via `setAttribute()`,
  //! detailed in the documentation for the corresponding enum value.
  enum class Type {
    //! A cylinder with rounded ends.
    //!
    //! The capsule is oriented along the z-axis.
    //!
    //! Origin: Geometric center, with the radius in the x-y plane, and spheres placed equidistant
    //!         along the z-axis.
    //!
    //! Attributes:
    //! - RADIUS: Radius of the capsule.
    //!           Default: 0.5
    //! - HEIGHT: Height of the capsule, defined to be the distance between the centers of the two
    //!           spheres defining the capsule.
    //!           Default: 1.0
    CAPSULE,

    //! An axis-aligned cuboid (i.e., rectangular prism).
    //!
    //! The cuboid is constructed to be aligned with the axes of its local coordinate frame.
    //!
    //! Origin: Geometric center.
    //!
    //! Attributes:
    //! - SIDE_LENGTHS: Dimensions along each axis (in the local obstacle frame). Each value
    //!                 represents the full extent (i.e., "length") of the cuboid, not the
    //!                 half-length.
    //!                 Default: {1.0, 1.0, 1.0}
    CUBOID,

    //! A sphere.
    //!
    //! Origin: The center of the sphere.
    //!
    //! Attributes:
    //! - RADIUS: The radius of the sphere.
    //!           Default: 0.5
    SPHERE,

    //! An axis-aligned uniform grid of signed distances.
    //!
    //! Origin: Center of the grid.
    //!
    //! Attributes:
    //! - GRID: Defines the dimensions and resolution of the SDF.
    //!         A grid precision of `Grid::Precision::DOUBLE` is unsupported and will result in a
    //!         fatal error if specified.
    SDF
  };

  //! `Attribute`s are used to modify obstacles from their default geometry.
  //!
  //! Each `Attribute` may be applicable to one or more `Obstacle::Type`s. Details about which
  //! `Attribute` can be used with which `Obstacle::Type` are included in documentation for
  //! `Obstacle::Type`.
  //!
  //! Each `Attribute` has a required type for the `AttributeValue` used in conjunction with
  //! `setAttribute()`, detailed in the documentation for each enum value below.
  enum class Attribute {
    //! The height of an obstacle (typically aligned with the z-axis).
    //!
    //! Data Type: `double`
    HEIGHT,

    //! The radius of an obstacle.
    //!
    //! Data Type: `double`
    RADIUS,

    //! The dimensions of the obstacle along the cardinal axes.
    //!
    //! Data Type: `Eigen::Vector3d`
    SIDE_LENGTHS,

    //! An axis-aligned regular grid of points.
    //!
    //! Data Type: `Obstacle::Grid`
    GRID
  };

  //! Used to specify the defining attributes for a grid of voxels that covers an axis-aligned
  //! rectangular region of the workspace.
  //!
  //! `Grid` fully describes how the grid values passed to `World::setGridValuesFromHost()` and
  //! `World::setGridValuesFromDevice()` will be interpreted.  An `Attribute::GRID` specifies a grid
  //! of voxels that each contain a scalar value.  Each voxel has a fixed workspace position
  //! associated with its value that is implicitly defined by the parameters in `Grid`.
  struct CUMO_EXPORT Grid {
    //! Floating-point precision of grid data.
    //!
    //! Underlying integer values correspond to the size of each floating-point type in bytes.
    enum class Precision {
      HALF = 2,
      FLOAT = 4,
      DOUBLE = 8
    };

    //! A `Grid` covers a rectangular region in the workspace whose minimal coordinate is
    //! the origin and whose voxel positions are determined by `num_voxels_x`, `num_voxels_y`,
    //! `num_voxels_z`, and `voxel_size`.
    //!
    //! The values associated with a `Grid` correspond to the center of each voxel.  The minimal
    //! corner of the minimal voxel in a `Grid` rests at the origin.  I.e., the origin is not at
    //! a voxel center; it is at a voxel corner.
    //!
    //! Voxels in a `Grid` have length `voxel_size` along each axis.
    //!
    //! Grid contains `num_voxels_x` voxels along the X dimension, `num_voxels_y` voxels along the Y
    //! dimension, and `num_voxels_z` voxels along the Z dimension.  This implies that the maximal
    //! position in the region of the workspace that contains a `Grid` is
    //! `voxel_size * [num_voxels_x, num_voxels_y, num_voxels_z]`.
    //!
    //! When grid values are set for an obstacle, up to two copies of the data will be maintained
    //! at separately specified levels of precision.  For example, it is possible to pass a set of
    //! grid values of type `double` and to specify that a set of grid values of type `double`
    //! should be kept in host (CPU) memory for use in distance queries made on the host, along with
    //! a set of grid values of type `float` kept resident in device (GPU) memory for use in
    //! distance queries made on the device.
    Grid(int num_voxels_x,
         int num_voxels_y,
         int num_voxels_z,
         double voxel_size,
         Precision host_precision,
         Precision device_precision)
        : num_voxels_x(num_voxels_x),
          num_voxels_y(num_voxels_y),
          num_voxels_z(num_voxels_z),
          voxel_size(voxel_size),
          host_precision(host_precision),
          device_precision(device_precision) {}

    //! The number of voxels along the X dimension of `Grid`.
    int num_voxels_x;

    //! The number of voxels along the Y dimension of `Grid`.
    int num_voxels_y;

    //! The number of voxels along the Z dimension of `Grid`.
    int num_voxels_z;

    //! The size of the voxels along each dimension.
    double voxel_size;

    //! The type of data for storing a set of grid values on CPU.
    Precision host_precision;

    //! The type of data for storing a set of grid values on GPU.
    Precision device_precision;
  };

  //! Used to specify the value for a given `Attribute`.
  //!
  //! The required `AttributeValue` constructor for each `Attribute` is detailed in the
  //! documentation for `Attribute`.
  struct CUMO_EXPORT AttributeValue {
    //! Create `AttributeValue` from `double`.
    AttributeValue(double value);  // NOLINT Allow implicit conversion

    //! Create `AttributeValue` from `Eigen::Vector3d`.
    AttributeValue(const Eigen::Vector3d &value);  // NOLINT Allow implicit conversion

    //! Create `AttributeValue` from `Grid`.
    AttributeValue(const Grid &value);  // NOLINT Allow implicit conversion

    struct Impl;
    std::shared_ptr<Impl> impl;
  };

  virtual ~Obstacle() = default;

  //! Set attribute for obstacle.
  //!
  //! Available attributes are based on the `Obstacle::Type` and detailed in documentation for
  //! `Obstacle::Type`.
  //!
  //! Example usage:
  //!     my_sphere.setAttribute(Obstacle::Attribute::RADIUS, 3.0);
  //!     my_cuboid.setAttribute(Obstacle::Attribute::SIDE_LENGTHS, Eigen::Vector3d(2.0, 3.0, 1.0));
  virtual void setAttribute(Attribute attribute, const AttributeValue &value) = 0;

  //! Return the `Obstacle::Type` of this obstacle.
  [[nodiscard]] virtual Type type() const = 0;
};

//! Create an obstacle with the given `type`.
//!
//! Available attributes and default attribute values for the given `type` are included in the
//! documentation for `Obstacle::Type`.
CUMO_EXPORT std::unique_ptr<Obstacle> CreateObstacle(Obstacle::Type type);

}  // namespace cumotion
