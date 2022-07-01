/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_FIELD_HPP_
#define NVIDIA_GXF_DW_POD_FIELD_HPP_

#include <cstring>
#include <type_traits>
#include <utility>

#include "common/endian.hpp"
#include "common/fixed_string.hpp"
#include "common/fixed_vector.hpp"
#include "gxf/dw/pod/types.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// Type of the field name used in @ref BasicField. This defines an effective maximum size of a
/// field name, although the DW Pod format supports a significantly larger value.
using FieldNameBuffer = FixedString<128U>;

/// An owned version of @ref EncodedFieldView. This keeps track of a name, value, and encoded
/// version of that value.
///
/// @tparam TLeaf The leaf type of field, such as @ref NumericField or @ref StringField.
///               This type should contain two
///               static functions: <tt>static size_t encode(Span<uint8_t>, ValueType const&)</tt>,
///               which encodes the provided value into the buffer and returns the encoded size;
///               and <tt>static ValueType decode(Span<uint8_t const>)</tt>, which decodes the
///               value from the buffer.
/// @tparam TValue The type of value this encodes.
/// @tparam KEncodedCapacity The maximum size of the buffer for the encoded value.
template <typename TLeaf, typename TValue, size_t KEncodedCapacity>
class BasicField {
  static_assert(std::is_nothrow_move_assignable<TValue>::value,
                "TValue must have noexcept move-assignment");

 public:
    using ValueType = TValue;

 public:
  /// Construct a field with a name constructed with @a nameArg and value constructed with
  /// @a valueArg. This overload only participates when @c TNameInit and @c TValueInit can
  /// be passed as arguments to construct @c FieldNameBuffer and @c ValueType, respectively.
  template <typename TNameInit,
            typename TValueInit,
            typename = std::enable_if_t<std::is_constructible<FieldNameBuffer, TNameInit>::value>,
            typename = std::enable_if_t<std::is_constructible<ValueType, TValueInit>::value>>
  explicit BasicField(TNameInit&& nameArg, TValueInit&& valueArg)
      : name_(std::forward<TNameInit>(nameArg)), value_(std::forward<TValueInit>(valueArg)) {
    encodeValue();
  }

  /// Decode the given @a src into an owned field. This will throw any exception
  /// @c TLeaf::decode can throw.
  explicit BasicField(EncodedFieldView src) : value_(TLeaf::decode(src.value)) {
    name_.copy(src.name.data(), src.name.size());
    buffer_.resize(src.value.size());
    std::memcpy(buffer_.data(), src.value.data(), src.value.size());
  }

  /// @{
  /// Copy operations assume that copying the value will not change its encoding.
  BasicField(BasicField const&) = default;
  BasicField& operator=(BasicField const&) = default;
  /// @}

  /// @{
  /// Move operations are only enabled if the backing @c ValueType has a trivial move operation.
  /// A trivial move operation guarantees it is equivalent to a copy, which is more strict than
  /// needed, but the actual check of "would the encoding change after a move" does not exist
  /// in C++.
  BasicField(BasicField&&) = default;
  BasicField& operator=(BasicField&&) = default;
  /// @}

  ~BasicField() = default;

  /// Get the name of this field.
  FieldNameBuffer const& getName() const noexcept { return name_; }

  /// Get the value stored in this field.
  ValueType const& getValue() const noexcept { return value_; }

  /// Get the encoded view of this field.
  EncodedFieldView asView() const & noexcept {
    return EncodedFieldView{Span<char const>{name_.data(), name_.size()},
                            Span<uint8_t const>{buffer_.data(), buffer_.size()}};
  }

  EncodedFieldView asView() && = delete;

 private:
  void encodeValue() {
    buffer_.resize(buffer_.capacity());
    auto encodedSz = TLeaf::encode(Span<uint8_t>(buffer_.data(), buffer_.size()), value_);
    buffer_.resize(encodedSz);
  }

 private:
  FieldNameBuffer name_;
  ValueType value_;
  FixedVector<std::uint8_t, KEncodedCapacity> buffer_;
};

/// Test that @a lhs and @a rhs are equal in name and value. This function is @c noexcept if the
/// @c TLeaf::ValueType has a @c noexcept equality operator.
template <typename TLeaf, typename TValue, size_t KEncodedCapacity>
bool operator==(BasicField<TLeaf, TValue, KEncodedCapacity> const& lhs,
                BasicField<TLeaf, TValue, KEncodedCapacity> const& rhs)
    noexcept(noexcept(lhs.getName() == rhs.getName() && lhs.getValue() == rhs.getValue())) {
  return lhs.getName() == rhs.getName() && lhs.getValue() == rhs.getValue();
}

/// Test that @a lhs and @a rhs are inequal by their name or value. This function is @c noexcept
/// if the @c TLeaf::ValueType has a @c noexcept equality operator.
template <typename TLeaf, typename TValue, size_t KEncodedCapacity>
bool operator!=(BasicField<TLeaf, TValue, KEncodedCapacity> const& lhs,
                BasicField<TLeaf, TValue, KEncodedCapacity> const& rhs)
    noexcept(noexcept(!(lhs == rhs))) {
  return !(lhs == rhs);
}

/// A numeric field encodes an integer @c TValue as its little-endian bytes.
template <typename TValue>
class NumericField final : public BasicField<NumericField<TValue>, TValue, sizeof(TValue)> {
  using BaseType = BasicField<NumericField, TValue, sizeof(TValue)>;

  static_assert(std::is_integral<TValue>::value, "A NumericField must have a numeric value type");
  static_assert(!std::is_same<TValue, bool>::value, "A NumericField can not be a bool");

 public:
  using typename BaseType::ValueType;

 public:
  using BaseType::BaseType;

  static size_t encode(Span<uint8_t> target, const ValueType& value) {
    auto encoded = EncodeLittleEndian(value);
    std::memcpy(target.data(), &encoded, sizeof(encoded));
    return sizeof(encoded);
  }

  static ValueType decode(Span<uint8_t const> source) {
    return DecodeLittleEndian(*reinterpret_cast<const ValueType*>(source.data()));
  }
};

/// Boolean is encoded as a string i.e. "true" or "false" with no null-terminator.
/// 5 bytes ensures we have enough space to encode the field.
constexpr size_t ENCODED_BOOLEAN_FIELD_SIZE = 5U;

/// A boolean field which encodes its value as either a @c "true" or @c "false" string.
class BooleanField final : public BasicField<BooleanField, bool, ENCODED_BOOLEAN_FIELD_SIZE> {
  using BaseType = BasicField<BooleanField, bool, ENCODED_BOOLEAN_FIELD_SIZE>;

 public:
  using typename BaseType::ValueType;

 public:
  using BaseType::BaseType;

  /// Encode the @a value as a string in @a target.
  static size_t encode(Span<uint8_t> target, const ValueType& value);

  /// Decode the @a source string. Any string starting with @c t, @c T, or @c 1
  /// is considered @c true, while @c f, @c F, or @c 0 are @c false.
  /// All other characters will cause @ref InvalidArgumentException to be thrown.
  static ValueType decode(Span<uint8_t const> source);
};

/// A string field encodes a string. This does not derive from @ref BasicField, as copying a
/// string to a different buffer is not useful.
///
/// @tparam TValue The type of string to use.
template <typename TValue = FieldNameBuffer>
class StringField final {
 public:
  using ValueType = TValue;

 public:
  /// Construct a field with a name constructed with @a nameArg and value constructed with
  /// @a valueArg. This overload only participates when @c TNameInit and @c TValueInit can
  /// be passed as arguments to construct @c FieldNameBuffer and @c ValueType, respectively.
  template <typename TNameInit,
            typename TValueInit,
            typename = std::enable_if_t<std::is_constructible<FieldNameBuffer, TNameInit>::value>,
            typename = std::enable_if_t<std::is_constructible<ValueType, TValueInit>::value>>
  explicit StringField(TNameInit&& nameArg, TValueInit&& valueArg)
      : name_(std::forward<TNameInit>(nameArg)), value_(std::forward<TValueInit>(valueArg)) {}

  /// Extract @a src into an owned field. This will throw if the size of the name or value
  /// exceeds the buffer sizes of this type.
  explicit StringField(EncodedFieldView src) {
    name_.copy(src.name.data(), src.name.size());
    value_.copy(reinterpret_cast<char const*>(src.value.data()), src.value.size());
  }

  StringField() = default;

  StringField(StringField const&) = default;
  StringField& operator=(StringField const&) = default;

  StringField(StringField&&) = default;
  StringField& operator=(StringField&&) = default;

  ~StringField() = default;

  FieldNameBuffer const& getName() const noexcept { return name_; }

  ValueType const& getValue() const noexcept { return value_; }

  EncodedFieldView asView() const & {
    // the "encoding" is just moving to an unsigned byte string
    auto encodedValue = Span<uint8_t const>{reinterpret_cast<std::uint8_t const*>(value_.data()),
                                            value_.size()};

    return EncodedFieldView{Span<char const>{name_.data(), name_.size()}, encodedValue};
  }

  EncodedFieldView asView() && = delete;

 private:
  FieldNameBuffer name_;
  ValueType value_;
};

template <typename TValue>
bool operator==(StringField<TValue> const& lhs, StringField<TValue> const& rhs) noexcept {
  return lhs.getName() == rhs.getName() && lhs.getValue() == rhs.getValue();
}

template <typename TValue>
bool operator!=(StringField<TValue> const& lhs, StringField<TValue> const& rhs) noexcept {
  return !(lhs == rhs);
}

/// Contains well-known field names.
struct KnownFields {
  template <typename TLeaf, typename TValue, template <typename> class TEncodePattern>
  struct BasicInfo {
    using ValueType = TValue;

    using FieldType = TEncodePattern<TValue>;

    /// Create a field with the given @a value. The name is provided by @c TLeaf::NAME.
    static FieldType make(ValueType const& value) {
      return FieldType(FieldNameBuffer(TLeaf::NAME.data(), TLeaf::NAME.size()), value);
    }
  };

  /// A 64-bit bitfield used to mark some characteristic of the frame.
  /// Non-presence of this field means the flag value is 0.
  struct Flags : public BasicInfo<Flags, std::uint64_t, NumericField> {
    static Span<char const> const NAME;
  };

  /// The string denoting the source of stamps.
  /// This is a comma-separated list of @ref dwTimeType values encoded with @c getName.
  struct StampTypes : public BasicInfo<StampTypes, FieldNameBuffer, StringField> {
    static Span<char const> const NAME;
  };
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_FIELD_HPP_
