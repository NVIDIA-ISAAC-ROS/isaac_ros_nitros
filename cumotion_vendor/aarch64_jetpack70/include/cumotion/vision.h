// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES.
//                         All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.

//! @file
//! @brief Public interface for handling depth camera data
//!
//! @note This interface is experimental and may evolve in a future release.  It also lacks
//!       a corresponding Python API.

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>

#include "Eigen/Core"

#include "cumotion/cumotion_export.h"

namespace cumotion {

//! Camera intrinsic parameters.
//!
//! The intrinsics define the mapping from 3D camera coordinates to 2D image coordinates. The
//! standard pinhole camera model uses four parameters (`fx`, `fy`, `cx`, `cy`) that form the
//! intrinsic matrix:
//!
//! @code
//!   [fx   0  cx]
//!   [ 0  fy  cy]
//!   [ 0   0   1]
//! @endcode
//!
//! where `fx` and `fy` are the focal lengths in pixels, and `cx` and `cy` are the principal point
//! coordinates (optical center) in pixels.
class CUMO_EXPORT CameraIntrinsics {
 public:
  //! Construct from the four standard pinhole camera parameters.
  //!
  //! The parameters `fx` and `fy` represent the focal lengths in the x and y directions
  //! (in pixels), while `cx` and `cy` represent the principal point (optical center) coordinates.
  CameraIntrinsics(double fx, double fy, double cx, double cy);

  //! Construct from a full 3x3 intrinsic matrix.
  explicit CameraIntrinsics(const Eigen::Matrix3d &matrix);

  //! Return the focal length in the x direction (in pixels).
  [[nodiscard]] double fx() const;

  //! Return the focal length in the y direction (in pixels).
  [[nodiscard]] double fy() const;

  //! Return the x coordinate of the principal point (optical center) in pixels.
  [[nodiscard]] double cx() const;

  //! Return the y coordinate of the principal point (optical center) in pixels.
  [[nodiscard]] double cy() const;

  //! Return the full 3x3 intrinsic matrix.
  [[nodiscard]] Eigen::Matrix3d matrix() const;

  //! Return whether the intrinsic matrix has the standard pinhole camera structure.
  //!
  //! Returns `true` if the matrix follows the canonical pinhole format with zero skew, zeros in the
  //! lower triangular entries, and 1 in the bottom-right corner. This allows for optimized
  //! operations that only use the four key parameters (`fx`, `fy`, `cx`, `cy`).
  [[nodiscard]] bool isPinhole() const;

  struct Impl;
  std::shared_ptr<Impl> impl;
};

//! Memory residency for buffer data.
enum class BufferResidency {
  //! Host memory allocated via `malloc()`, `new`, `cudaMallocHost()`, etc.
  HOST,

  //! Device memory allocated via `cudaMalloc()` or similar CUDA device allocators.
  DEVICE,

  //! CUDA unified/managed memory allocated via `cudaMallocManaged()`.
  //!
  //! This memory is accessible from both host and device, with migration handled as needed by the
  //! CUDA runtime.
  MANAGED
};

//! Type-agnostic base class for depth images and (non-owning) views of depth images.
//!
//! See `DepthImage` for member function documentation.
class CUMO_EXPORT DepthImageBase {
 public:
  //! Scalar type used to represent the depth values.
  enum class ScalarType {
    FLOAT,  //!< 32-bit floating point (corresponds to `DepthImage<float>`)
    UINT16  //!< 16-bit unsigned integer (corresponds to `DepthImage<uint16_t>`)
  };

  virtual ~DepthImageBase() = default;

  [[nodiscard]] virtual int width() const = 0;
  [[nodiscard]] virtual int height() const = 0;
  [[nodiscard]] virtual int stride() const = 0;
  [[nodiscard]] virtual BufferResidency residency() const = 0;
  [[nodiscard]] virtual double metersPerUnit() const = 0;
  [[nodiscard]] virtual bool isView() const = 0;
  [[nodiscard]] virtual ScalarType scalarType() const = 0;
  [[nodiscard]] virtual bool isConst() const = 0;
  [[nodiscard]] virtual double depthInMeters(int x, int y) const = 0;
};

//! A depth image or non-owning view of a depth image buffer.
//!
//! The data buffer is assumed to be contiguous with pixels in the conventional row-major order
//! (with the x coordinate varying fastest) and with the origin at the top-left corner.
//!
//! Supported types are `float` and `uint16_t`.  The latter is often used for raw depth data where
//! distance is measured in millimeters.  Const-qualified variants are also supported.
//!
//! The `stride()` will always be equal to or greater than `width()`.  The pixel at coordinates
//! (x, y) is located at index `y * stride() + x` in the buffer, where `x` is the column index and
//! `y` is the row index.
template<typename T = float>
class DepthImage : public DepthImageBase {
  using BaseT = std::remove_const_t<T>;
  static_assert(std::is_same_v<BaseT, float> || std::is_same_v<BaseT, uint16_t>,
                "'DepthImage' supports 'float' and 'uint16_t' types.");

 public:
  //! Return the default `meters_per_unit` value for the scalar type `T`:
  //!
  //! - `0.001` for `uint16_t`, corresponding to units of millimeters.
  //! - `1.0` for `float`, corresponding to units of meters.
  [[nodiscard]] static constexpr double DefaultMetersPerUnit() {
    return std::is_same_v<const T, const uint16_t> ? 0.001 : 1.0;
  }

  //! Construct a `DepthImage` that owns its own buffer.
  //!
  //! This constructor is only available for non-const scalar types (`float` or `uint16_t`).
  //!
  //! The optional `stride` parameter specifies the number of elements between consecutive rows. If
  //! not provided, the stride defaults to `width` (tightly-packed layout). The `residency`
  //! parameter indicates where to allocate the buffer, and the `meters_per_unit` parameter
  //! specifies the scaling factor to convert depth values to meters.
  //!
  //! A fatal error will be logged if:
  //!
  //! 1. `width` or `height` is zero or negative, or
  //! 2. `stride` is smaller than `width`.
  template<typename U = T, typename = std::enable_if_t<!std::is_const_v<U>>>
  DepthImage(int width, int height,
             std::optional<int> stride = std::nullopt,
             BufferResidency residency = BufferResidency::HOST,
             double meters_per_unit = DefaultMetersPerUnit());

  ~DepthImage() override = default;

  DepthImage(DepthImage&&) noexcept = default;
  DepthImage& operator=(DepthImage&&) noexcept = default;

  // Copying is disallowed.
  DepthImage(const DepthImage&) = delete;
  DepthImage& operator=(const DepthImage&) = delete;

  //! Return pointer to the underlying depth data buffer.
  [[nodiscard]] T *data();

  //! Return const pointer to the underlying depth data buffer.
  [[nodiscard]] const T *data() const;

  //! Return the width of the depth image in pixels.
  [[nodiscard]] int width() const override;

  //! Return the height of the depth image in pixels.
  [[nodiscard]] int height() const override;

  //! Return the stride of the depth image in elements.
  //!
  //! The stride represents the number of elements between consecutive rows in the buffer. For
  //! tightly-packed images, stride equals width. For padded or aligned images, stride may be
  //! larger than width.
  [[nodiscard]] int stride() const override;

  //! Return the memory residency of the depth image buffer.
  [[nodiscard]] BufferResidency residency() const override;

  //! Return the scaling factor to convert depth values to meters.
  //!
  //! This value represents the number of meters per depth unit. Common values include 1.0 for
  //! depth already in meters, 0.001 for millimeters, and 0.0001 for tenths of millimeters.
  [[nodiscard]] double metersPerUnit() const override;

  //! Return whether this is a (non-owning) view or a `DepthImage` that owns its memory.
  //!
  //! Views do not manage the lifetime of the underlying buffer. Owning instances allocate and
  //! deallocate the buffer automatically.
  [[nodiscard]] bool isView() const override;

  //! Return the scalar type of the depth image data (needed for `DepthImageBase` interface).
  [[nodiscard]] ScalarType scalarType() const override {
    using BaseT = std::remove_const_t<T>;
    return std::is_same_v<BaseT, float> ? ScalarType::FLOAT : ScalarType::UINT16;
  }

  //! Return whether the depth image data is const-qualified (needed for `DepthImageBase`
  //! interface).
  [[nodiscard]] bool isConst() const override {
    return std::is_const_v<T>;
  }

  //! Return the depth value at pixel coordinates (x, y).
  //!
  //! The returned value is the raw depth measurement without unit conversion. The `x` coordinate
  //! corresponds to the column index, and `y` corresponds to the row index, with the origin at
  //! the top-left corner.
  //!
  //! @warning This accessor does not perform bounds checking and does not check that the data is
  //!          resident on the host.
  [[nodiscard]] T at(int x, int y) const;

  //! Return a reference to the depth value at pixel coordinates (x, y).
  //!
  //! This function allows modification of the depth value and is only available for non-const
  //! template instantiations. The `x` coordinate corresponds to the column index, and `y`
  //! corresponds to the row index, with the origin at the top-left corner.
  //!
  //! @warning This accessor does not perform bounds checking and does not check that the data is
  //!          resident on the host.
  [[nodiscard]] T &at(int x, int y);

  //! Return the depth value at pixel coordinates (`x`, `y`) converted to meters.
  //!
  //! This method returns the depth value scaled by `metersPerUnit()`. The `x` coordinate
  //! corresponds to the column index, and `y` corresponds to the row index, with the origin at
  //! the top-left corner.
  //!
  //! A fatal error will be logged if the data is not resident on the host or if either `x` or `y`
  //! is out of bounds.
  [[nodiscard]] double depthInMeters(int x, int y) const override;

  struct Impl;

  //! Constructor used by the internal implementation.
  explicit DepthImage(std::shared_ptr<Impl> impl);

 private:
  std::shared_ptr<Impl> impl_;
};

extern template class CUMO_EXTERN_TEMPLATE_EXPORT DepthImage<float>;
extern template class CUMO_EXTERN_TEMPLATE_EXPORT DepthImage<const float>;
extern template class CUMO_EXTERN_TEMPLATE_EXPORT DepthImage<uint16_t>;
extern template class CUMO_EXTERN_TEMPLATE_EXPORT DepthImage<const uint16_t>;

//! Create a non-owning `DepthImage` view from a depth data buffer.
//!
//! The template parameter `T` can be `float`, `uint16_t`, `const float`, or `const uint16_t`.
//!
//! The `data` parameter points to a contiguous buffer containing depth values arranged in
//! row-major order. The buffer must remain valid for the lifetime of the returned `DepthImage`
//! object, as the view does not take ownership of the memory.
//!
//! The optional `stride` parameter specifies the number of elements between consecutive rows. If
//! not provided, the stride defaults to `width` (tightly-packed layout). The `residency`
//! parameter indicates where the buffer is allocated, and the `meters_per_unit` parameter
//! specifies the scaling factor to convert depth values to meters.
//!
//! A fatal error will be logged if:
//!
//! 1. `data` is null,
//! 2. `width` or `height` is zero or negative, or
//! 3. `stride` is smaller than `width`.
template<typename T>
CUMO_EXPORT DepthImage<T> CreateDepthImageView(
  T *data,
  int width,
  int height,
  std::optional<int> stride = std::nullopt,
  BufferResidency residency = BufferResidency::HOST,
  double meters_per_unit = DepthImage<T>::DefaultMetersPerUnit());

}  // namespace cumotion
