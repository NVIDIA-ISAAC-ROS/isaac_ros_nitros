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
#include <utility>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/buffer.hpp"
#include "engine/core/byte.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// -------------------------------------------------------------------------------------------------

// A pixel which owns its data and can life independently from the image itself.
template <typename K, int N>
using Pixel = std::conditional_t<N == 1, K, Vector<K, N>>;

// A pixel type used for read and write access to an image pixel.
template <typename K, int N>
using PixelRef = std::conditional_t<N == 1, K&, Eigen::Map<Vector<K, N>>>;

// A pixel type used for read-only access to an image pixel.
template <typename K, int N>
using PixelConstRef = std::conditional_t<N == 1, const K&, Eigen::Map<const Vector<K, N>>>;

namespace detail {

// Helper type to create pixels and pixel references based on a pointer for N > 1
template <typename K, int N>
struct PixelCreator {
  // Creates a non-owning pixel based on a pointer
  static PixelRef<K, N> CreatePixelRef(K* ptr) {
    return PixelRef<K, N>(ptr);
  }
  // Creates a non-owning, non-mutable pixel based on a pointer
  static PixelConstRef<K, N> CreatePixelConstRef(const K* ptr) {
    return PixelConstRef<K, N>(ptr);
  }
};

// Specialized version of PixelCreator for N = 1
template <typename K>
struct PixelCreator<K, 1> {
  static PixelRef<K, 1> CreatePixelRef(K* ptr) {
    return *ptr;
  }
  static PixelConstRef<K, 1> CreatePixelConstRef(const K* ptr) {
    return *ptr;
  }
};

// Helper type to find the correct tensor type based on the number of channels.
template <typename K, int N, typename BufferType>
struct ImageTensorBase {
  using type = TensorBase<K, tensor::Dimensions<tensor::kDynamic, tensor::kDynamic, N>, BufferType>;
};
template <typename K, typename BufferType>
struct ImageTensorBase<K, 1, BufferType> {
  using type = TensorBase<K, tensor::Rank<2>, BufferType>;
};

}  // namespace detail

// -------------------------------------------------------------------------------------------------

// A two-dimensional array of pixels
//
// Each pixel contains `N` elements and all elements are of the same type `K`. Elements for all
// pixels are stored continuously. Access to pixels returns mapped memory to provide read access
// without unnecessary copies and direct write access. Pixels are stored interleaved and in
// row-major storage order.
//
// The image type uses a tensor to store data. If `N` is equal to 1 a rank 2 tensor is used,
// otherwise a rank 3 tensor is used. The tensor uses the given `BufferType` and more information
// is provided in the Tensor type.
//
template <typename K, int N, typename BufferType>
class ImageBase {
 public:
  // The number of channels for an image must be greater than 0.
  static_assert(N > 0, "Number of channels must be positive");
  // The number of channels for each pixels. This is also the last dimension of the underlying
  // tensor.
  static constexpr int kChannels = N;

  // Tensor type used to store the array of elements for this image.
  using tensor_t = typename detail::ImageTensorBase<K, kChannels, BufferType>::type;

  using buffer_t = BufferType;
  using buffer_view_t = typename tensor_t::buffer_view_t;
  using buffer_const_view_t = typename tensor_t::buffer_const_view_t;
  static constexpr bool kIsMutable = tensor_t::kIsMutable;
  static constexpr bool kIsOwning = tensor_t::kIsOwning;

  using index_t = typename tensor_t::index_t;
  using coordinate_t = Vector2<index_t>;  // Note: Only rows and col are used for coordinates
  using dimensions_t = Vector2<index_t>;  // Note: Only rows and col are used for dimensions

  using element_t = typename tensor_t::element_t;
  using element_ptr_t = typename tensor_t::element_ptr_t;
  using element_const_ptr_t = typename tensor_t::element_const_ptr_t;

  using raw_ptr_t = typename tensor_t::raw_ptr_t;
  using raw_const_ptr_t = typename tensor_t::raw_const_ptr_t;

  // Type used for a pixel owning its memory and thus able to life outside of this image
  using pixel_t = Pixel<K, kChannels>;
  // Type used when accessing pixels which provides read and write access to pixel data.
  using pixel_ref_t = PixelRef<K, kChannels>;
  // Type used when accessing pixels which provides only read access to pixel data.
  using pixel_const_ref_t = PixelConstRef<K, kChannels>;

  // Type for a mutable view on this image
  using image_view_t = ImageBase<K, kChannels, buffer_view_t>;
  // Type for a non-mutable view on this image
  using image_const_view_t = ImageBase<K, kChannels, buffer_const_view_t>;

  // Create an empty image object
  ImageBase() = default;

  // Create an image object of given dimensions. This will create a new storage container and is
  // only available for storage container types which own their memory. This function is templated
  // on the integer type so that we can distable this constructor for non-owning storage types.
  ImageBase(const dimensions_t& dimensions)
      : tensor_(dimensions[0], dimensions[1]) { }
  ImageBase(index_t rows, index_t cols)
      : tensor_(rows, cols) { }

  // Create an image object of given dimensions and with given storage container
  ImageBase(buffer_t buffer, const dimensions_t& dimensions)
      : tensor_(std::move(buffer), dimensions[0], dimensions[1]) { }
  ImageBase(buffer_t buffer, index_t rows, index_t cols)
      : tensor_(std::move(buffer), rows, cols) { }

  // Creates an image from a tensor.
  ImageBase(tensor_t tensor)
      : tensor_(std::move(tensor)) { }

  // Copy construction uses the default behavior
  ImageBase(const ImageBase& other) = default;
  // Copy assignment uses the default behavior
  ImageBase& operator=(const ImageBase& other) = default;
  // Move construction uses the default behavior
  ImageBase(ImageBase&& other) = default;
  // Move assignment uses the default behavior
  ImageBase& operator=(ImageBase&& other) = default;

  // Creates a mutable view on this tensor
  template <bool X = kIsMutable>
  std::enable_if_t<X, image_view_t> view() {
    return image_view_t({this->data().begin(), this->data().size()}, this->dimensions());
  }
  // Creates a non-mutable view on this tensor
  image_const_view_t const_view() const {
    return image_const_view_t({this->data().begin(), this->data().size()}, this->dimensions());
  }
  image_const_view_t view() const {
    return const_view();
  }

  // Provides conversion to a mutable view for owning images and mutable views.
  template <bool X = kIsMutable>
  operator std::enable_if_t<X, image_view_t>() { return view(); }
  // Provides conversion to a non-mutable view
  operator image_const_view_t() const { return const_view(); }

  // Returns true if this image has dimensions 0
  bool empty() const { return rows() == 0 || cols() == 0; }
  // (rows, cols) as a 2-vector
  dimensions_t dimensions() const { return dimensions_t{rows(), cols()}; }
  // Returns the number of rows
  index_t rows() const { return tensor().dimensions()[0]; }
  // Returns the number of cols
  index_t cols() const { return tensor().dimensions()[1]; }
  // The number of channels
  constexpr int channels() const { return kChannels; }
  // The total number of pixels in the image
  index_t num_pixels() const { return rows() * cols(); }

  // Resizes the image
  template <bool X = kIsOwning>
  std::enable_if_t<X, void> resize(index_t desired_rows, index_t desired_cols) {
    tensor_.resize(desired_rows, desired_cols);
  }
  template <bool X = kIsOwning>
  std::enable_if_t<X, void> resize(const dimensions_t& dimensions) {
    tensor_.resize(dimensions[0], dimensions[1]);
  }

  // Returns true if the given pixel coordinate references a valid pixel.
  bool isValidCoordinate(index_t row, index_t col) const {
    return 0 <= row && row < rows() && 0 <= col && col < cols();
  }

  // Returns a non mutable reference pixel holder on the position (row, col).
  pixel_const_ref_t operator()(index_t row, index_t col) const {
    return detail::PixelCreator<K, kChannels>::CreatePixelConstRef(
        row_pointer(row) + col * kChannels);
  }
  pixel_const_ref_t operator()(const coordinate_t& coordinate) const {
    return this->operator()(coordinate[0], coordinate[1]);
  }
  // Returns a mutable reference pixel holder on the position (row, col).
  template <bool X = kIsMutable>
  std::enable_if_t<X, pixel_ref_t> operator()(index_t row, index_t col) {
    return detail::PixelCreator<K, kChannels>::CreatePixelRef(
        row_pointer(row) + col * kChannels);
  }
  template <bool X = kIsMutable>
  std::enable_if_t<X, pixel_ref_t> operator()(const coordinate_t& coordinate) {
    return this->operator()(coordinate[0], coordinate[1]);
  }

  // Returns a non mutable reference pixel holder at the position `index`.
  pixel_const_ref_t operator[](index_t index) const {
    return detail::PixelCreator<K, kChannels>::CreatePixelConstRef(
        element_wise_begin() + index * kChannels);
  }
  // Returns a mutable reference pixel holder at the position `index`.
  template <bool X = kIsMutable>
  std::enable_if_t<X, pixel_ref_t> operator[](index_t index) {
    return detail::PixelCreator<K, kChannels>::CreatePixelRef(
        element_wise_begin() + index * kChannels);
  }

  // Pointers to the first element of the first pixel
  element_const_ptr_t element_wise_begin() const { return tensor().element_wise_begin(); }
  element_ptr_t element_wise_begin() { return tensor().element_wise_begin(); }
  // Pointers behind the last element of the last pixel
  element_const_ptr_t element_wise_end() const { return tensor().element_wise_end(); }
  element_ptr_t element_wise_end() { return tensor().element_wise_end(); }
  // The total number of elements in the image
  index_t num_elements() const { return num_pixels() * channels(); }

  // Pointer to the first element in the i-th row in an image
  element_ptr_t row_pointer(index_t row) {
    return element_wise_begin() + row * cols() * kChannels;
  }
  element_const_ptr_t row_pointer(index_t row) const {
    return element_wise_begin() + row * cols() * kChannels;
  }

  // Number of bytes used to store one row of the image in memory.
  size_t getStride() const {
    return ByteCount(1, cols());
  }
  // Number of bytes which are used for elements of an image of the given number of rows and columns
  static size_t ByteCount(index_t rows, index_t cols) {
    ASSERT(rows >= 0, "Number of rows must not be negative: %d", rows);
    ASSERT(cols >= 0, "Number of cols must not be negative: %d", cols);
    // Note: Number of channels are guaranteed to be positive.
    return static_cast<size_t>(rows * cols * kChannels) * sizeof(K);
  }

  // The underlying tensor object
  const tensor_t& tensor() const { return tensor_; }
  template <bool X = kIsMutable>
  std::enable_if_t<X, tensor_t&> tensor() { return tensor_; }

  // The underlying buffer object
  const buffer_t& data() const { return tensor().data(); }
  template <bool X = kIsMutable>
  std::enable_if_t<X, buffer_t&> data() { return tensor().data(); }

 private:
  tensor_t tensor_;
};

// -------------------------------------------------------------------------------------------------

// An image which owns it's memory
template <typename K, int N>
using Image = ImageBase<K, N, CpuBuffer>;

// A mutable view on an image which does not own memory but can be used to read and write the
// data of the underlying image.
template <typename K, int N>
using ImageView = ImageBase<K, N, CpuBufferView>;

// A non-mutable view on an image which does not own the memory and can only be used to read
// the data of the underlying image.
template <typename K, int N>
using ImageConstView = ImageBase<K, N, CpuBufferConstView>;

#define ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, T, S)   \
  using Image##N##S          = Image<T, N>;           \
  using ImageView##N##S      = ImageView<T, N>;       \
  using ImageConstView##N##S = ImageConstView<T, N>;  \
  using Pixel##N##S          = Pixel<T, N>;           \
  using PixelRef##N##S       = PixelRef<T, N>;        \
  using PixelConstRef##N##S  = PixelConstRef<T, N>;   \

#define ISAAC_DECLARE_CPU_IMAGE_TYPES(N)                                 \
  template <typename K> using Image##N = Image<K, N>;                    \
  template <typename K> using ImageView##N = ImageView<K, N>;            \
  template <typename K> using ImageConstView##N = ImageConstView<K, N>;  \
  ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, uint8_t,  ub)                    \
  ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, uint16_t, ui16)                  \
  ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, int,      i)                     \
  ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, double,   d)                     \
  ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL(N, float,    f)                     \

ISAAC_DECLARE_CPU_IMAGE_TYPES(1)
ISAAC_DECLARE_CPU_IMAGE_TYPES(2)
ISAAC_DECLARE_CPU_IMAGE_TYPES(3)
ISAAC_DECLARE_CPU_IMAGE_TYPES(4)

#undef ISAAC_DECLARE_CPU_IMAGE_TYPES
#undef ISAAC_DECLARE_CPU_IMAGE_TYPES_IMPL

// -------------------------------------------------------------------------------------------------
// Helper function to create an image view from a pointer using dense storage
template <typename K, int N>
ImageView<K, N> CreateImageView(K* data, typename ImageView<K, N>::index_t rows,
                                typename ImageView<K, N>::index_t cols) {
  ASSERT(rows >= 0, "Number of rows must not be negative - was %d", rows);
  ASSERT(cols >= 0, "Number of columns must not be negative - was %d", cols);
  const size_t size = ImageView<K, N>::ByteCount(rows, cols);
  return ImageView<K, N>(CpuBufferView(reinterpret_cast<byte*>(data), size), rows, cols);
}

// Helper function to create an image const view from a pointer using dense storage
template <typename K, int N>
ImageConstView<K, N> CreateImageConstView(const K* data,
                                          typename ImageConstView<K, N>::index_t rows,
                                          typename ImageConstView<K, N>::index_t cols) {
  ASSERT(rows >= 0, "Number of rows must not be negative - was %d", rows);
  ASSERT(cols >= 0, "Number of columns must not be negative - was %d", cols);
  const size_t size = ImageConstView<K, N>::ByteCount(rows, cols);
  return ImageConstView<K, N>(CpuBufferConstView(reinterpret_cast<const byte*>(data), size),
                              rows, cols);
}

// -------------------------------------------------------------------------------------------------

// An image stored in device memory which owns it's memory
template <typename K, int N>
using CudaImage = ImageBase<K, N, CudaBuffer>;

// A mutable view on an image which is stored on GPU device memory, does not own memory, but can be
// used to read and write the data of the underlying image.
template <typename K, int N>
using CudaImageView = ImageBase<K, N, CudaBufferView>;

// A non-mutable view on an image which is stored on GPU device memory, does not own its memory, and
// can only be used to read the data of the underlying image.
template <typename K, int N>
using CudaImageConstView = ImageBase<K, N, CudaBufferConstView>;

// Helper macro for ISAAC_DECLARE_CUDA_IMAGE_TYPES
#define ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, K, S)          \
  using CudaImage##N##S          = CudaImage<K, N>;           \
  using CudaImageView##N##S      = CudaImageView<K, N>;       \
  using CudaImageConstView##N##S = CudaImageConstView<K, N>;  \

// Helper macro to define various CudaImage types
#define ISAAC_DECLARE_CUDA_IMAGE_TYPES(N)                                        \
  template <typename K> using CudaImage##N          = CudaImage<K, N>;           \
  template <typename K> using CudaImageView##N      = CudaImageView<K, N>;       \
  template <typename K> using CudaImageConstView##N = CudaImageConstView<K, N>;  \
  ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, uint8_t,  ub)                           \
  ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, uint16_t, ui16)                         \
  ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, int,      i)                            \
  ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, double,   d)                            \
  ISAAC_DECLARE_CUDA_IMAGE_TYPES_IMPL(N, float,    f)                            \

ISAAC_DECLARE_CUDA_IMAGE_TYPES(1)
ISAAC_DECLARE_CUDA_IMAGE_TYPES(2)
ISAAC_DECLARE_CUDA_IMAGE_TYPES(3)
ISAAC_DECLARE_CUDA_IMAGE_TYPES(4)

// -------------------------------------------------------------------------------------------------
// Helper function to create a CUDA image view from a pointer using dense storage
template <typename K, int N>
CudaImageView<K, N> CreateCudaImageView(K* data, typename CudaImageView<K, N>::index_t rows,
                                        typename CudaImageView<K, N>::index_t cols) {
  ASSERT(rows >= 0, "Number of rows must not be negative - was %d", rows);
  ASSERT(cols >= 0, "Number of columns must not be negative - was %d", cols);
  const size_t size = CudaImageView<K, N>::ByteCount(rows, cols);
  return CudaImageView<K, N>(CudaBufferView(reinterpret_cast<byte*>(data), size), rows, cols);
}

// Helper function to create a CUDA image const view from a pointer using dense storage
template <typename K, int N>
CudaImageConstView<K, N> CreateCudaImageConstView(const K* data,
                                                  typename CudaImageConstView<K, N>::index_t rows,
                                                  typename CudaImageConstView<K, N>::index_t cols) {
  ASSERT(rows >= 0, "Number of rows must not be negative - was %d", rows);
  ASSERT(cols >= 0, "Number of columns must not be negative - was %d", cols);
  const size_t size = CudaImageConstView<K, N>::ByteCount(rows, cols);
  return CudaImageConstView<K, N>(CudaBufferConstView(reinterpret_cast<const byte*>(data), size),
                                  rows, cols);
}

// -------------------------------------------------------------------------------------------------

}  // namespace isaac
