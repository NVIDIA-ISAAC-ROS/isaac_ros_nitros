/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_BIT_FLAGS_HPP_
#define NVIDIA_GXF_DW_POD_BIT_FLAGS_HPP_

#include "common/type_utils.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// The base type to use for flags-like values. It contains the common operations
/// used on flags: @c |, @c &, @c -, and element inclusion @ref isSet and @ref isAnySet.
/// The primary use case for this class is cases where integer constants
/// or enumeration types are used as bit flags (hence the name).
///
/// @code
/// // Create the actual type deriving from BitFlags. By convention, this class is named ____Type.
/// struct ToppingType final : BitFlags<ToppingType, uint16_t> {
///   using BitFlags<ToppingType, uint16_t>::BitFlags;
/// };
///
/// // Create the variants in a separate class as a series of static constexpr values.
/// struct Topping final {
///   static constexpr ToppingType kNone{0x0000};
///   static constexpr ToppingType kRedSauce{0x0001};
///   static constexpr ToppingType kWhiteSauce{0x0002};
///   static constexpr ToppingType kCheese{0x0004};
///   static constexpr ToppingType kPepperoni{0x0008};
///   static constexpr ToppingType kGreenPepper{0x0010};
///   static constexpr ToppingType kMushrooms{0x0020};
///   static constexpr ToppingType kOlives{0x0040};
///   static constexpr ToppingType kChives{0x0080};
///   static constexpr ToppingType kPineapple{0x0100};
///   static constexpr ToppingType kAll{0x01ff};
/// };
/// @endcode
///
/// If that code lives in a header, you @e might get link errors for symbols like
/// @c Topping::kPepperoni (depending on if the symbols are referred to through references
/// and if those references get optimized out). This is because an inline defined static
/// constant does not get a symbol emitted for it, but references to the symbol are still
/// legal to create (even though they can't exist). This C++ language-level bug is fixed
/// in C++17 with the "inline variables" feature. Until then, this can be worked around by
/// defining these spaces in the cpp file:
///
/// @code
/// constexpr ToppingType Topping::kNone;
/// constexpr ToppingType Topping::kRedSauce;
/// constexpr ToppingType Topping::kWhiteSauce;
/// constexpr ToppingType Topping::kCheese;
/// constexpr ToppingType Topping::kPepperoni;
/// constexpr ToppingType Topping::kGreenPepper;
/// constexpr ToppingType Topping::kMushrooms;
/// constexpr ToppingType Topping::kOlives;
/// constexpr ToppingType Topping::kChives;
/// constexpr ToppingType Topping::kPineapple;
/// constexpr ToppingType Topping::kAll;
/// @endcode
///
/// @note
/// The unary @c ~ operator is not defined by this type. Unless the variants fully cover the
/// underlying numeric representation, values returned by a naive operator @c ~ will include
/// nonsensical values. For example, <tt>~(Topping::kWhiteSauce | Topping::kPineapple)</tt> would
/// have the makings of a pretty good pizza, but toppings covered by @c 0xfe00 would be confusing.
/// If you choose to implement unary @c ~, it is a good idea to narrow the result to only the
/// valid constants.
///
/// @tparam TReal The "real" leaf type of bit flags. This is the type returned from the operators.
/// @tparam TValue The backing value type. It must be some integer type.
template <typename TReal, typename TValue>
class BitFlags {
 public:
  static_assert(IsIntegral_v<TValue>, "TValue must be an integral type (uint32_t, int16_t, etc)");

  /// The type of value which backs this type.
  using value_type = TValue;

  explicit constexpr BitFlags() = default;

  /// Create an instance with the given @a value.
  explicit constexpr BitFlags(TValue value) : value_(value) {}

  constexpr TValue get() const { return value_; }

  /// Test that this instance contains all the elements of @a mask.
  ///
  /// @param mask The bits to test for. If this is empty (@ref value_ is @c 0),
  ///             this will always return @c true.
  ///
  /// @see isAnySet
  constexpr bool isSet(BitFlags mask) const { return (*this & mask) == mask; }

  /// Test if this instance contains any of the elements of @a mask.
  ///
  /// @param mask The bits to test for. If this is empty (@ref value_ is @c 0),
  ///             this will always return @c false.
  ///
  /// @see isSet
  constexpr bool isAnySet(BitFlags mask) const {
    return (*this & mask) != TReal{static_cast<TValue>(0)};
  }

 private:
  /// \anchor value_ - the unwrapped value.
  TValue value_;
};

/// @{
/// Set union operator. The result contains everything in either @a lhs or @a rhs.
template <typename TReal, typename TValue>
constexpr TReal operator|(BitFlags<TReal, TValue> lhs, BitFlags<TReal, TValue> rhs) {
  return TReal{static_cast<TValue>(lhs.get() | rhs.get())};
}

/// @see operator|
template <typename TReal, typename TValue>
constexpr TReal& operator|=(TReal& lhs, BitFlags<TReal, TValue> rhs) {
  lhs = (lhs | rhs);
  return lhs;
}
/// @}

/// @{
/// Set intersection operator. The result contains only things in both @a lhs and @a rhs.
template <typename TReal, typename TValue>
constexpr TReal operator&(BitFlags<TReal, TValue> lhs, BitFlags<TReal, TValue> rhs) {
  return TReal{static_cast<TValue>(lhs.get() & rhs.get())};
}

/// @see operator&
template <typename TReal, typename TValue>
constexpr TReal& operator&=(TReal& lhs, BitFlags<TReal, TValue> rhs) {
  lhs = (lhs & rhs);
  return lhs;
}
/// @}

/// @{
/// Set difference operator. The result contains bits set in @a lhs that were not set in @a rhs.
template <typename TReal, typename TValue>
constexpr TReal operator-(BitFlags<TReal, TValue> lhs, BitFlags<TReal, TValue> rhs) {
  return TReal{static_cast<TValue>(lhs.get() & static_cast<TValue>(~rhs.get()))};
}

/// @see operator-
template <typename TReal, typename TValue>
constexpr TReal& operator-=(TReal& lhs, BitFlags<TReal, TValue> rhs) {
  lhs = (lhs - rhs);
  return lhs;
}
/// @}

/// @{
/// Test that @a lhs and @a rhs have the same value.
template <typename TReal, typename TValue>
constexpr bool operator==(BitFlags<TReal, TValue> lhs, BitFlags<TReal, TValue> rhs) {
  return lhs.get() == rhs.get();
}

/// Test that @a lhs and @a rhs do not have the same value.
template <typename TReal, typename TValue>
constexpr bool operator!=(BitFlags<TReal, TValue> lhs, BitFlags<TReal, TValue> rhs) {
  return !(lhs == rhs);
}
/// @}

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_BIT_FLAGS_HPP_
