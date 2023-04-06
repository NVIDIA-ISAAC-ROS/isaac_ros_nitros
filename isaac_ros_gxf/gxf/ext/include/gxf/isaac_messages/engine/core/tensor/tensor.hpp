/*
Copyright (c) 2019-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <initializer_list>
#include <type_traits>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/buffer.hpp"
#include "engine/core/byte.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// -------------------------------------------------------------------------------------------------

namespace tensor {
namespace details {

// A compile-time sequence of integer
template <int... I>
using int_sequence = std::integer_sequence<int, I...>;

// Helper type to define repeated_int_sequence
template <int N, int V, int... D>
struct repeated_int_sequence_impl {
    using type = typename repeated_int_sequence_impl<N - 1, V, V, D...>::type;
};
template <int V, int... D>
struct repeated_int_sequence_impl<0, V, D...> {
    using type = int_sequence<D...>;
};

// A compile-time sequence of integers which are all identical
template <int N, int V>
using repeated_int_sequence = typename repeated_int_sequence_impl<N, V>::type;

template <typename IntegerSequence>
struct DropFirstImpl;

template <int First, int... Tail>
struct DropFirstImpl<int_sequence<First, Tail...>> {
  using type = int_sequence<Tail...>;
};

// Compute the logical AND for a list of arguments
template<typename First, typename... Rest>
struct VariadicAnd : std::integral_constant<bool, First::value && VariadicAnd<Rest...>::value> {};
template<typename First>
struct VariadicAnd<First> : std::integral_constant<bool, First::value> {};

}  // namespace details

// This value can be used for tensor dimensions to indicate that the size is not fixed at compile
// time and will instead be provided at runtime. We use the same value as Eigen to be compatible.
constexpr int kDynamic = Eigen::Dynamic;

// A compile-time list of integers used to specify tensor dimensions.
template <int... I>
using Dimensions = details::int_sequence<I...>;

// A compile-time list of integers used to specify tensor dimensions which are all dynamic.
template <int N>
using Rank = details::repeated_int_sequence<N, kDynamic>;

template <typename IntegerSequence>
using DropFirst = typename details::DropFirstImpl<IntegerSequence>::type;

// Gets the i-th element in an integer sequence
template<int... Ints>
constexpr int IntSequenceAt(Dimensions<Ints...>, int i) {
    constexpr int arr[] = {Ints...};
    return i < 0 ? 0 : arr[i];
}

// Gets the last element in an integer sequence
template<int... Ints>
constexpr int IntSequenceLast(Dimensions<Ints...>) {
    constexpr int arr[] = {Ints...};
    return arr[sizeof...(Ints) - 1];
}

// Gets a vector of dimensions based on the compile-time dimensions and the dimensions given as
// arguments. If the compile-time dimension is dynamic (-1) the dimension passed as an argument
// will be used instead. Otherwise the compile-time dimension must match the argument dimension.
// Dimensions must not be negative with the exception of using -1 for compile-time dimensions. It is
// also possible to obmit trailing argument dimensions if they are fixed at compile time.
template <int... Dimensions, typename... Args>
Vector<int, sizeof...(Dimensions)> GetDimensionsVector(Args... dimensions) {
  constexpr int kRank = sizeof...(Dimensions);
  constexpr int kArgumentCount = sizeof...(Args);
  std::array<int, kRank> compile_time_dimensions{Dimensions...};
  Vector<int, kArgumentCount> runtime_dimensions(dimensions...);
  Vector<int, kRank> result;
  for (int i = 0; i < kRank; i++) {
    if (i >= kArgumentCount) {
      // The dimension was not specified as an argument. Thus use the compile-time dimension
      ASSERT(compile_time_dimensions[i] != -1, "Dimension argument can only be obmitted if the "
             "dimension is specified at compile-time");
      ASSERT(compile_time_dimensions[i] > 0, "Dimension argument can only be obmitted if the "
             "compile-time dimension is positive. Was: %d", compile_time_dimensions[i]);
      result[i] = compile_time_dimensions[i];
    } else {
      // The dimension was specified as an argument. Use if it the compile-time dimension is -1.
      // Otherwise they both need to argree.
      ASSERT(compile_time_dimensions[i] == -1 ||
             compile_time_dimensions[i] == runtime_dimensions[i],
             "dimension mismatch: %d vs %d", compile_time_dimensions[i], runtime_dimensions[i]);
      ASSERT(runtime_dimensions[i] >= 0, "dimensions must not be negative: %d", result[i]);
      result[i] = runtime_dimensions[i];
    }
  }
  return result;
}

// Gets a vector of dimensions based the template parameters. For a dimension of dynamic size a
// size of zero will be chosen.
template <int... Dimensions>
Vector<int, sizeof...(Dimensions)> GetDimensionsVector() {
  constexpr int kRank = sizeof...(Dimensions);
  std::array<int, kRank> compile_time_dimensions{Dimensions...};
  Vector<int, kRank> result;
  for (int i = 0; i < kRank; i++) {
    if (compile_time_dimensions[i] == kDynamic) {
      result[i] = 0;
    } else {
      ASSERT(compile_time_dimensions[i] > 0, "Compile-time dimensions must be positive or Dynamic "
             "Was: %d", compile_time_dimensions[i]);
      result[i] = compile_time_dimensions[i];
    }
  }
  return result;
}

}  // namespace tensor

// -------------------------------------------------------------------------------------------------

template <typename K, typename Dimensions, typename Buffer>
struct TensorBase;

// A multi-dimensional array of elements.
//
// All elements are of the same type `K`.
//
// The tensor stores a multi-dimensional array of elements. The sizes of the array are specified by
// the template parameter `Dimensions`. An std::integer_sequence of ints is used as type for the
// list of dimensions. For each axes the dimension can either be a positive integer which indicates
// a compile-time dimension or it can be `tensor::Dynamic` (which is -1) which indicates that the
// dimension is specified at runtime. The template `tensor::Dimensions<...>` can be used as an alias
// for `std::integer_sequence<int, ...>`. The template `tensor::Rank<N>` can be used as an alias for
// `std::integer_sequence<int, tensor::Dynamic, ... /*(N times)*/>`.
//
// Elements are stored in a buffer of type `Buffer`. Most notably buffers can either own memory or
// just be a view on memory owned by someone else. Buffers can also either allow or not allow
// modifications of elements.
template <typename K, typename Buffer, int... DimensionsPack>
class TensorBase<K, tensor::details::int_sequence<DimensionsPack...>, Buffer> {
 public:
  // Tensor<const int, ...> is not allowed. Use TensorConstView<int, ...> instead.
  static_assert(!std::is_const<K>::value, "Tensor can only be used with non-const element type");

  // The integer sequence containing the sizes of the tensor
  using Dimensions = tensor::details::int_sequence<DimensionsPack...>;
  // The rank of the tensor is the number of its dimensions.
  static constexpr int kRank = sizeof...(DimensionsPack);
  // The rank must be greater than 0.
  static_assert(kRank > 0, "Rank must be at least 1");
  // @deprecated
  static constexpr int Order = kRank;

  // The underlying buffer object used to store elements.
  using buffer_t = Buffer;
  // Type for mutable views on the underlying buffer
  using buffer_view_t = typename BufferTraits<buffer_t>::buffer_view_t;
  // Type for non-mutable views on the underlying buffer
  using buffer_const_view_t = typename BufferTraits<buffer_t>::buffer_const_view_t;
  // A mutable tensor allows modification of elements.
  static constexpr bool kIsMutable = BufferTraits<buffer_t>::kIsMutable;
  // An owning tensor owns its memory while a non-owning memory uses memory owned by another party.
  static constexpr bool kIsOwning = BufferTraits<buffer_t>::kIsOwning;

  // The integral type used for a single row or column index.
  using index_t = int;
  // The vector type to use for pixel coordinates.
  using coordinate_t = Vector<index_t, kRank>;
  // The vector type to use for pixel dimensions.
  using dimensions_t = Vector<index_t, kRank>;

  using element_t = std::remove_cv_t<K>;
  using element_const_ptr_t = std::add_const_t<element_t>*;
  using element_ptr_t = std::conditional_t<kIsMutable, element_t*, element_const_ptr_t>;

  using element_const_ref_t = std::add_const_t<element_t>&;
  using element_ref_t = std::conditional_t<kIsMutable, element_t&, element_const_ref_t>;

  using raw_const_ptr_t = std::add_const_t<byte>*;
  using raw_ptr_t = std::conditional_t<kIsMutable, byte*, raw_const_ptr_t>;

  using tensor_view_t = TensorBase<K, Dimensions, buffer_view_t>;
  using tensor_const_view_t = TensorBase<K, Dimensions, buffer_const_view_t>;

  TensorBase() {
    // If no dimensions are provided set to zero.
    setDimensions(tensor::GetDimensionsVector<DimensionsPack...>());
  }

  // Constructs a tensor with the given dimensions which owns its data.
  TensorBase(const dimensions_t& dimensions) {
    resize(dimensions, true);
  }
  template <typename... RequestedDimensionsPack,
            std::enable_if_t<tensor::details::VariadicAnd<
                std::is_convertible<RequestedDimensionsPack, int>...>::value>* = nullptr>
  TensorBase(RequestedDimensionsPack... dimensions)
      : TensorBase(tensor::GetDimensionsVector<DimensionsPack...>(dimensions...)) {}

  // Constructs a tensor based on an existing buffer object using the given dimensions. This
  // constructor will throw if the provided buffer does not contain enough data.
  TensorBase(buffer_t data, const dimensions_t& dimensions)
      : data_(std::move(data)) {
    ASSERT(CheckDimensions(dimensions), "Trying to create tensor with invalid dimensions");
    setDimensions(dimensions);
    ASSERT(data_.size() >= byte_size(), "Buffer too small: provided %zd, required %zd",
           data_.size(), byte_size());
  }
  template <typename... RequestedDimensionsPack,
            std::enable_if_t<tensor::details::VariadicAnd<
                std::is_convertible<RequestedDimensionsPack, int>...>::value>* = nullptr>
  TensorBase(buffer_t data, RequestedDimensionsPack... dimensions)
      : TensorBase(std::move(data), tensor::GetDimensionsVector<DimensionsPack...>(dimensions...)) {
  }

  // Copy construction uses the default behavior
  TensorBase(const TensorBase& other) = default;
  // Copy assignment uses the default behavior1
  TensorBase& operator=(const TensorBase& other) = default;
  // Move construction uses the default behavior
  TensorBase(TensorBase&& other) {
    *this = std::move(other);
  }
  // Move assignment uses the default behavior
  TensorBase& operator=(TensorBase&& other) {
    data_ = std::move(other.data_);
    // The dimensions of the tensor.
    dimensions_ = other.dimensions_;
    offsets_ = other.offsets_;
    other.dimensions_.setZero();
    other.offsets_.setZero();
    return *this;
  }

  // Creates a mutable view on this tensor
  template <bool X = kIsMutable>
  std::enable_if_t<X, tensor_view_t> view() {
    return tensor_view_t({this->data().begin(), this->data().size()}, this->dimensions());
  }
  // Creates a non-mutable view on this tensor
  tensor_const_view_t const_view() const {
    return tensor_const_view_t({this->data().begin(), this->data().size()}, this->dimensions());
  }

  // Provides conversion to a mutable view for owning tensors and mutable views.
  template <bool X = kIsMutable>
  operator std::enable_if_t<X, tensor_view_t>() { return view(); }
  // Provides conversion to a non-mutable view
  operator tensor_const_view_t() const { return const_view(); }

  // Returns true if any dimension of this tensor is 0 and thus if the number of elements stored in
  // this tensor is 0.
  bool empty() const { return element_count() == 0; }
  // Returns the rank of the tensor
  constexpr int rank() const { return kRank; }
  // Deprecated - use rank() instead
  constexpr int order() const { return kRank; }
  // Returns the dimensions of the tensor
  dimensions_t dimensions() const { return dimensions_; }

  // Resizes the tensor. This operation is destructive.
  template <bool X = kIsOwning>
  std::enable_if_t<X, void> resize(const dimensions_t& dimensions,
                                   bool force_reallocation = false) {
    // Only allocate new memory if the dimensions change.
    if (force_reallocation || dimensions != dimensions_) {
      ASSERT(CheckDimensions(dimensions), "Invalid dimensions");
      setDimensions(dimensions);
      data_ = buffer_t(byte_size());
    }
  }

  // Similar as `resize` but takes dimensions individually without using the vector type.
  template <typename... RequestedDimensionsPack, bool X = kIsMutable,
            std::enable_if_t<X>* = nullptr>
  void resize(RequestedDimensionsPack... requested_dimensions) {
    return resize(tensor::GetDimensionsVector<DimensionsPack...>(requested_dimensions...));
  }

  // Returns true if the element coordinate is within the tensor's dimension
  bool isValidCoordinate(const coordinate_t& indicies) const {
    return (indicies.array() >= 0).all() && (indicies.array() < dimensions_.array()).all();
  }

  // Helper wrappers for isValidCoordinate for the most common orders of tensors
  template <typename... Args,
      std::enable_if_t<sizeof...(Args) == kRank>* = nullptr>
  bool isValidCoordinate(Args... indices) const {
    return isValidCoordinate(coordinate_t(indices...));
  }

  // Accesses the given index in the tensor with respect to memory ordering
  template <bool X = std::is_trivial<element_t>::value>
  std::enable_if_t<X, element_t> operator()(const coordinate_t& indicies) const {
    return *(this->element_wise_begin() + indexToOffset(indicies));
  }
  // Accesses the given index in the tensor with respect to memory ordering
  template <bool X = !std::is_trivial<element_t>::value>
  std::enable_if_t<X, element_const_ref_t> operator()(const coordinate_t& indicies) const {
    return *(this->element_wise_begin() + indexToOffset(indicies));
  }

  // Accesses the given index in the tensor with respect to memory ordering
  template <bool X = kIsMutable>
  std::enable_if_t<X, element_ref_t> operator()(const coordinate_t& indicies) {
    return *(this->element_wise_begin() + indexToOffset(indicies));
  }

  // Helper wrappers for operator access for the most common orders of tensors
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank &&
                             std::is_trivial<element_t>::value>* = nullptr>
  element_t operator()(Args... indices) const {
    return operator()(coordinate_t(indices...));
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank &&
                             !std::is_trivial<element_t>::value>* = nullptr>
  element_const_ref_t operator()(Args... indices) const {
    return operator()(coordinate_t(indices...));
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank && kIsMutable>* = nullptr>
  element_ref_t operator()(Args... indices) {
    return operator()(coordinate_t(indices...));
  }

  // Creates a view on a slice with one dimension less. The dimension with highest significance
  // is sliced.
  template <bool X = kIsMutable && kRank >= 2>
  std::enable_if_t<X, TensorBase<K, tensor::DropFirst<Dimensions>, buffer_view_t>>
  slice(index_t index) {
    using slice_t = TensorBase<K, tensor::DropFirst<Dimensions>, buffer_view_t>;
    const size_t slice_size = static_cast<size_t>(offsets_[0]) * sizeof(K);
    const size_t slice_offset = static_cast<size_t>(index) * slice_size;
    return slice_t(buffer_view_t{this->data().begin() + slice_offset, slice_size},
                   this->dimensions().template segment<kRank - 1>(1).eval());
  }
  template <bool X = !kIsMutable && kRank >= 2>
  std::enable_if_t<X, TensorBase<K, tensor::DropFirst<Dimensions>, buffer_const_view_t>>
  slice(index_t index) {
    return const_slice(index);
  }

  // Creates a const view on a slice with one dimension less. The dimension with highest
  // significance is sliced.
  template <bool X = kRank >= 2>
  std::enable_if_t<X, TensorBase<K, tensor::DropFirst<Dimensions>, buffer_const_view_t>>
  const_slice(index_t index) const {
    using const_slice_t = TensorBase<K, tensor::DropFirst<Dimensions>, buffer_const_view_t>;
    const size_t slice_size = static_cast<size_t>(offsets_[0]) * sizeof(K);
    const size_t slice_offset = static_cast<size_t>(index) * slice_size;
    return const_slice_t(buffer_const_view_t{this->data().begin() + slice_offset, slice_size},
                         this->dimensions().template segment<kRank - 1>(1).eval());
  }

  // Helper types for Eigen matrix-style access
  static constexpr int kBackDim2 = tensor::IntSequenceAt(Dimensions{}, kRank - 2);
  static constexpr int kBackDim1 = tensor::IntSequenceAt(Dimensions{}, kRank - 1);
  using matrix_view_t = Eigen::Map<Eigen::Matrix<K, kBackDim2, kBackDim1, Eigen::RowMajor>>;
  using matrix_const_view_t = Eigen::Map<
      const Eigen::Matrix<K, kBackDim2, kBackDim1, Eigen::RowMajor>>;
  using vector_view_t = Eigen::Map<Vector<K, kBackDim1>>;
  using vector_const_view_t = Eigen::Map<const Vector<K, kBackDim1>>;

  // Returns a mutable Eigen matrix view sliced on the last two dimensions
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank - 2 && kIsMutable>* = nullptr>
  matrix_view_t matrix(Args... indices) {
    return matrix_view_t(
        this->element_wise_begin() + indexToOffset(coordinate_t(indices..., 0, 0)),
        dimensions_[kRank - 2], dimensions_[kRank - 1]);
  }
  // Returns a non-mutable Eigen matrix view sliced on the last two dimensions
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank - 2>* = nullptr>
  matrix_const_view_t matrix(Args... indices) const {
    return matrix_const_view_t(
        this->element_wise_begin() + indexToOffset(coordinate_t(indices..., 0, 0)),
        dimensions_[kRank - 2], dimensions_[kRank - 1]);
  }
  // Returns a mutable Eigen vector view sliced on the last dimension
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank - 1 && kIsMutable>* = nullptr>
  vector_view_t matrix(Args... indices) {
    return vector_view_t(
        this->element_wise_begin() + indexToOffset(coordinate_t(indices..., 0)),
        dimensions_[kRank - 1]);
  }
  // Returns a non-mutable Eigen vector view sliced on the last dimension
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank - 1>* = nullptr>
  vector_const_view_t matrix(Args... indices) const {
    return vector_const_view_t(
        this->element_wise_begin() + indexToOffset(coordinate_t(indices..., 0)),
        dimensions_[kRank - 1]);
  }

  // When indices are omitted in operator() access it behaves the same as `matrix()`.
  template <typename... Args,
            std::enable_if_t<(sizeof...(Args) == kRank - 2 ||
                             sizeof...(Args) == kRank - 1) && kIsMutable>* = nullptr>
  auto operator()(Args... indices) { return matrix(indices...); }
  // When indices are omitted in operator() access it behaves the same as `matrix()`.
  template <typename... Args,
            std::enable_if_t<sizeof...(Args) == kRank - 2 ||
                             sizeof...(Args) == kRank - 1>* = nullptr>
  auto operator()(Args... indices) const { return matrix(indices...); }

  // Const Pointer to the beginning of the data block
  element_const_ptr_t element_wise_begin() const {
    return reinterpret_cast<element_const_ptr_t>(data_.begin());
  }
  // Pointer to the beginning of the data block
  element_ptr_t element_wise_begin() {
    return reinterpret_cast<element_ptr_t>(data_.begin());
  }
  // Const Pointer to the beginning of the data block
  element_const_ptr_t element_wise_end() const {
    return reinterpret_cast<element_const_ptr_t>(data_.end());
  }
  // Pointer to the beginning of the data block
  element_ptr_t element_wise_end() {
    return reinterpret_cast<element_ptr_t>(data_.end());
  }
  // The total number of elements in the tensor
  index_t num_elements() const { return offsets_[0] * dimensions_[0]; }
  // @deprecated
  int element_count() const { return num_elements(); }

  // Returns a non mutable reference pixel holder at the position `index`.
  element_const_ref_t operator[](index_t index) const {
    return this->element_wise_begin()[index];
  }
  // Returns a mutable reference pixel holder at the position `index`.
  template <bool X = kIsMutable>
  std::enable_if_t<X, element_ref_t> operator[](index_t index) {
    return this->element_wise_begin()[index];
  }

  // const access to the underlying buffer object
  const buffer_t& data() const { return data_; }
  // access to the underlying buffer object
  template <bool X = kIsMutable>
  std::enable_if_t<X, buffer_t&> data() { return data_; }
  // The total numbe of bytes required to store the tensor
  size_t byte_size() const {
    return static_cast<size_t>(element_count()) * sizeof(element_t);
  }

 private:
  // Returns true if all dimensions are greater than zero or zero
  static bool CheckDimensions(const dimensions_t& dimensions) {
    return (dimensions.array() >= 0).all();
  }

  // Sets the dimensions of the tensor and updates offsets.
  void setDimensions(const dimensions_t& dimensions) {
    dimensions_ = dimensions;
    offsets_[kRank - 1] = 1;  // Rank is guaranteed to be positive
    for (int i = kRank - 1; i > 0; i--) {
      offsets_[i - 1] = offsets_[i] * dimensions_[i];
    }
  }

  // Computes the position of an element based on its coordinate.
  int indexToOffset(const coordinate_t& indices) const {
    return offsets_.dot(indices);
  }

  // storage for the tensor
  buffer_t data_;
  // The dimensions of the tensor.
  dimensions_t dimensions_;
  // Dimension offsets for indexing rows of storage
  dimensions_t offsets_;
};  // namespace isaac

// -------------------------------------------------------------------------------------------------

template <typename K, int N>
using CpuTensor = TensorBase<K, tensor::Rank<N>, CpuBuffer>;

template <typename K, int N>
using CpuTensorView = TensorBase<K, tensor::Rank<N>, CpuBufferView>;

template <typename K, int N>
using CpuTensorConstView = TensorBase<K, tensor::Rank<N>, CpuBufferConstView>;

#define ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, T, S)          \
  using CpuTensor##N##S          = CpuTensor<T, N>;           \
  using CpuTensorView##N##S      = CpuTensorView<T, N>;       \
  using CpuTensorConstView##N##S = CpuTensorConstView<T, N>;  \

#define ISAAC_DECLARE_CPU_TENSOR_TYPES(N)                                  \
  template <typename K> using CpuTensor##N          = CpuTensor<K, N>;     \
  template <typename K> using CpuTensorView##N      = CpuTensorView<K, N>; \
  template <typename K> using CpuTensorConstView##N = CpuTensorConstView<K, N>;  \
  ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, uint8_t,  ub)                     \
  ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, uint16_t, ui16)                   \
  ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, int,      i)                      \
  ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, double,   d)                      \
  ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL(N, float,    f)                      \

ISAAC_DECLARE_CPU_TENSOR_TYPES(1)
ISAAC_DECLARE_CPU_TENSOR_TYPES(2)
ISAAC_DECLARE_CPU_TENSOR_TYPES(3)
ISAAC_DECLARE_CPU_TENSOR_TYPES(4)

#undef ISAAC_DECLARE_CPU_TENSOR_TYPES
#undef ISAAC_DECLARE_CPU_TENSOR_TYPES_IMPL

// -------------------------------------------------------------------------------------------------

// An Tensor stored in device memory which owns it's memory
template <typename K, int N>
using GpuTensor = TensorBase<K, tensor::Rank<N>, CudaBuffer>;

// A mutable view on an Tensor which is stored on GPU device memory, does not own memory, but can
// be used to read and write the data of the underlying Tensor.
template <typename K, int N>
using GpuTensorView = TensorBase<K, tensor::Rank<N>, CudaBufferView>;

// A non-mutable view on an Tensor which is stored on GPU device memory, does not own its memory,
// and can only be used to read the data of the underlying Tensor.
template <typename K, int N>
using GpuTensorConstView = TensorBase<K, tensor::Rank<N>, CudaBufferConstView>;

// Helper macro for ISAAC_DECLARE_CUDA_TENSOR_TYPES
#define ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, K, S)           \
  using GpuTensor##N##S          = GpuTensor<K, N>;           \
  using GpuTensorView##N##S      = GpuTensorView<K, N>;       \
  using GpuTensorConstView##N##S = GpuTensorConstView<K, N>;  \

// Helper macro to define various GpuTensor types
#define ISAAC_DECLARE_CUDA_TENSOR_TYPES(N)                                         \
  template <typename K> using GpuTensor##N          = GpuTensor<K, N>;           \
  template <typename K> using GpuTensorView##N      = GpuTensorView<K, N>;       \
  template <typename K> using GpuTensorConstView##N = GpuTensorConstView<K, N>;  \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, uint8_t,  ub)                            \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, uint16_t, ui16)                          \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, int,      i)                             \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, double,   d)                             \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, float,    f)                             \

ISAAC_DECLARE_CUDA_TENSOR_TYPES(1)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(2)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(3)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(4)

#undef ISAAC_DECLARE_CUDA_TENSOR_TYPES
#undef ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL

// -------------------------------------------------------------------------------------------------

// Creates a CpuTensorView instance from a raw host `data` pointer that allows to alter the data
// pointed to, although it will still be owned by the entity that allocated it originally. The
// `elements` parameter denotes the number of elements contained within `data`. The `shape`
// parameter is a `Vector<int, N>` with `N` being the tensor's rank. Its elements are the dimensions
// of the tensor. The template parameters denote the variable type the CpuTensorView should represent
// and the rank of the tensor.
template<typename K, int N>
CpuTensorView<K, N> CreateCpuTensorViewFromData(K* data, size_t elements, const Vector<int, N>& shape) {
  return CpuTensorView<K, N>(CpuBufferView(HostPointer<byte>(reinterpret_cast<byte*>(data)),
                                        sizeof(K) * elements), shape);
}

// Creates a TensorConstView instance from a raw host `data` pointer. The data pointed to will still
// be owned by the entity that allocated it originally. The `elements` parameter denotes the number
// of elements contained within `data`. The `shape` parameter is a `Vector<int, N>` with `N` being
// the tensor's rank. Its elements are the dimensions of the tensor. The template parameters denote
// the variable type the CpuTensorView should represent and the rank of the tensor.
template<typename K, int N>
CpuTensorConstView<K, N> CreateCpuTensorConstViewFromData(const K* data, size_t elements,
                                                    const Vector<int, N>& shape) {
  return CpuTensorConstView<K, N>(
      CpuBufferConstView(HostPointer<const byte>(reinterpret_cast<const byte*>(data)),
                         sizeof(K) * elements), shape);
}

// Creates a GpuTensorView instance from a raw device `data` pointer that allows to alter the data
// pointed to, although it will still be owned by the entity that allocated it originally. The
// `elements` parameter denotes the number of elements contained within `data`. The `shape`
// parameter is a `Vector<int, N>` with `N` being the tensor's rank. Its elements are the dimensions
// of the tensor. The template parameters denote the variable type the TensorView should represent
// and the rank of the tensor.
template<typename K, int N, typename T>
GpuTensorView<K, N> CreateGpuTensorViewFromData(T* data, size_t elements,
                                                  const Vector<int, N>& shape) {
  return GpuTensorView<K, N>(
      CudaBufferView(CudaPointer<byte>(reinterpret_cast<byte*>(data)),
                     sizeof(K) * elements), shape);
}

// Creates a GpuTensorConstView instance from a raw device `data` pointer. The data pointed to
// will still be owned by the entity that allocated it originally.The `elements` parameter
// denotes the number of elements contained within `data`. The `shape` parameter is a
// `Vector<int, N>` with `N` being the tensor's rank. Its elements are the dimensions of the
// tensor. The template parameters denote the variable type the TensorView should represent and
// the rank of the tensor.
template<typename K, int N, typename T>
GpuTensorConstView<K, N> CreateGpuTensorConstViewFromData(const T* data, size_t elements,
                                                            const Vector<int, N>& shape) {
  return GpuTensorConstView<K, N>(
      CudaBufferConstView(CudaPointer<const byte>(reinterpret_cast<const byte*>(data)),
                          sizeof(K) * elements), shape);
}

// -------------------------------------------------------------------------------------------------

}  // namespace isaac
