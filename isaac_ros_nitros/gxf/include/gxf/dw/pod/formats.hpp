/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_FORMATS_HPP_
#define NVIDIA_GXF_DW_POD_FORMATS_HPP_

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <tuple>

#include "common/endian.hpp"
#include "common/span.hpp"
#include "common/type_utils.hpp"
#include "gxf/dw/pod/data.hpp"
#include "gxf/dw/pod/types.hpp"

/// @file
/// Contains the binary format encoding and decoding functionality.
///
/// If you are trying to learn how the classes and functions of this file work, it is recommended
/// to read the generic @ref Format template first, then skip down to the "Specific Formats"
/// section to see how it all comes together. You can fill in the middle as needed.

namespace nvidia {
namespace gxf {
namespace pod {

using EncodeTarget = Span<uint8_t>;

using DecodeSource = Span<uint8_t const>;

/// The unspecialized template has all functions intentionally deleted. These functions are meant
/// to provide documentation for the specializations.
///
/// When implementing specializations for @c Format, partial specialization should generally be
/// avoided. Generic, reusable formats should have long-form names (@ref DynamicSizeArrayFormat,
/// @ref NamedTypeFormat, etc.), which are used to build explicit specializations for Pod-specific
/// types (@ref BundleId, @ref TrackHeader, etc.). An exception to this rule are fixed-size arrays,
/// as they have an unambiguous encoding; but even fixed-size arrays use the generic
/// @ref FixedSizeArrayFormat helper.
template <typename T>
struct Format {
  /// Get the encoded size of the given @a value. For most types, this is a fixed value,
  /// but for variable-sized lists, this can be based on the contents of @a value.
  static size_t encodedSize(FileVersion const& version, T const& value) = delete;

  /// Encode the given @a value into the @a target. The @a target will be sized by @ref encodedSize.
  ///
  /// @return The number of bytes actually written to @a target. For most cases, this value
  ///         will be the same number as what @ref encodedSize would return, but some formats
  ///         (such as @ref DynamicUnwritten) do not write the contents of their fields.
  static size_t encode(EncodeTarget target, FileVersion const& version, T const& value) = delete;

  /// Decode the @a source contents into the provided @a output.
  ///
  /// @return The number of bytes of @a source that were consumed.
  /// @throw std::runtime_error if the @a source does not look valid.
  static size_t decode(T& output, FileVersion const& version, DecodeSource source) = delete;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Format Building Blocks                                                                         //
//                                                                                                //
// This section contains generic Format implementations and building blocks for your own Formats. //
// Implementations for DW Pod-specific types like TrackId or TrackHeaderFrame belong in the       //
// "Specific Formats" section.                                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////

/// A trivial format which encodes a single byte \c T value directly into the target.
template <typename T>
struct SingleByteFormat {
  static_assert(sizeof(T) == 1U, "Must be a single byte");

  static constexpr size_t encodedSize(FileVersion const&, T const&) {
    return 1U;
  }

  static size_t encode(EncodeTarget target, FileVersion const&, T const& value) {
    target[0U].value() = uint8_t(value);
    return 1U;
  }

  static size_t decode(T& output, FileVersion const&, DecodeSource source) {
    output = T(source[0U].value());
    return 1U;
  }
};

template <>
struct Format<char> : SingleByteFormat<char> {};

template <typename T>
struct BasicNumericFormat {
  static constexpr size_t encodedSize(FileVersion const&, T const&) {
    return sizeof(T);
  }

  static size_t encode(EncodeTarget target, FileVersion const&, T const& value) {
    auto encoded = EncodeLittleEndian(value);
    std::memcpy(target.first(sizeof(encoded))->data(), &encoded, sizeof(encoded));
    return sizeof(encoded);
  }

  static size_t decode(T& output, FileVersion const&, DecodeSource source) {
    output = DecodeLittleEndian(*reinterpret_cast<const T*>(source.first(sizeof(T))->data()));
    return sizeof(T);
  }
};

template <> struct Format<uint64_t> : BasicNumericFormat<uint64_t> {};
template <> struct Format<int64_t>  : BasicNumericFormat<int64_t>  {};
template <> struct Format<uint32_t> : BasicNumericFormat<uint32_t> {};
template <> struct Format<int32_t>  : BasicNumericFormat<int32_t>  {};
template <> struct Format<uint16_t> : BasicNumericFormat<uint16_t> {};
template <> struct Format<int16_t>  : BasicNumericFormat<int16_t>  {};
template <> struct Format<uint8_t>  : BasicNumericFormat<uint8_t>  {};
template <> struct Format<int8_t>   : BasicNumericFormat<int8_t>   {};

/// A number with a C++ representation of @a TCpp, but encoded as @a TEncoded.
template <typename TCpp, typename TEncoded, typename TEncodedFormats = Format<TEncoded>>
struct NarrowedNumericFormat {
  static_assert(IsSigned_v<TCpp> == IsSigned_v<TEncoded>,
                "TCpp and TEncoded must have same signedness");
  static_assert(sizeof(TEncoded) < sizeof(TCpp), "TEncoded must be smaller than TCpp");

  static constexpr TEncoded downcast(TCpp const& value) {
    auto lo = static_cast<TCpp>(std::numeric_limits<TEncoded>::min());
    auto hi = static_cast<TCpp>(std::numeric_limits<TEncoded>::max());

    if (value < lo || value > hi) {
      throw std::invalid_argument("Provided value is outside of range of encoded type");
    } else {
      return static_cast<TEncoded>(value);
    }
  }

  static constexpr size_t encodedSize(FileVersion const& version, TCpp const& value) {
    return TEncodedFormats::encodedSize(version, downcast(value));
  }

  static size_t encode(EncodeTarget target, FileVersion const& version, TCpp const& value) {
    return TEncodedFormats::encode(target, version, downcast(value));
  }

  static size_t decode(TCpp& output, FileVersion const& version, DecodeSource source) {
    TEncoded encoded{};
    auto sz = TEncodedFormats::decode(encoded, version, source);

    // always safe -- TCpp is wider than TEncoded
    output = static_cast<TCpp>(encoded);
    return sz;
  }
};

/// Wrap a @a UnderlyingFormat by writing the encoded size with the given @a SizeFormat.
///
/// If @a UnderlyingFormat would have written the value as a 200 byte sequence, this format
/// would write a 208 byte sequence, with the first 8 bytes being `208`
/// (assuming the default @a SizeFormat with a @c uint64_t).
///
/// @tparam BackPadding The amount of padding after @c UnderlyingFormat encoding to add
///                     (but still include in the size). This is used by @ref FooterFrame
///                     because its encoder and decoder compute checksums on the
///                     encoded bytes which lay after the main content.
/// @tparam AllContentEncoded Does the @c UnderlyingFormat encode all the content?
///                           This is set to @c false by @ref TrackHeaderFrame and @ref DataFrame,
///                           which use @ref DynamicUnwritten members.
template <typename UnderlyingFormat,
          typename SizeFormat     = Format<uint64_t>,
          size_t BackPadding      = 0U,
          bool AllContentEncoded  = true>
struct WrapSizeFormat {
  template <typename TValue>
  static constexpr size_t encodedSize(FileVersion const& version, TValue const& value) {
    auto wrappedSz = UnderlyingFormat::encodedSize(version, value) + BackPadding;
    return wrappedSz + SizeFormat::encodedSize(version, wrappedSz);
  }

  template <typename TValue>
  static size_t encode(EncodeTarget target, FileVersion const& version, TValue const& value) {
    auto finalGoalSz = encodedSize(version, value);
    auto sizeGoalSz  = SizeFormat::encodedSize(version, finalGoalSz);

    auto sizeTarget    = target.subspan(0, sizeGoalSz).value();
    auto wrappedTarget = target.subspan(sizeGoalSz).value();

    auto wrappedSz = UnderlyingFormat::encode(wrappedTarget, version, value);
    auto sizeSz    = SizeFormat::encode(sizeTarget, version, finalGoalSz);

    auto finalSz = wrappedSz + sizeSz + BackPadding;
    if (AllContentEncoded && ((finalSz != finalGoalSz) || (sizeSz != sizeGoalSz))) {
      throw std::runtime_error("Mismatch between predicted sizes and actual encoded sizes");
    }

    return finalSz;
  }

  template <typename TValue>
  static size_t decode(TValue& output, FileVersion const& version, DecodeSource source) {
    size_t encodedSz{};
    auto sizeSz = SizeFormat::decode(encodedSz, version, source);

    // Get a subview of the source that does not include the encoded size at the front.
    // If all the content is encoded, it is also cut down at the back.
    // If not all the content is encoded, then we can't snip the back
    // because it wasn't included in the original source.
    auto internalSource = source.subspan(sizeSz).value();
    if (AllContentEncoded) {
      internalSource = internalSource.subspan(0U, encodedSz - sizeSz - BackPadding).value();
    }

    auto wrappedSz = UnderlyingFormat::decode(output, version, internalSource);

    if (AllContentEncoded && (sizeSz + wrappedSz + BackPadding != encodedSz)) {
      throw std::runtime_error("Malformed encoding of sized field");
    }

    return encodedSz;
  }
};

/// A formatter for @ref NamedType instances.
///
/// @tparam T The C++ type to encode or decode. While this is expected to be a
///           @ref NamedType of sorts, it will work for any type that has a @c .value member.
/// @tparam UnderlyingFormat The format to use to encode the value. The default value of encoding
///                          it exactly as the @c T::value_type is reasonable, but this can be
///                          paired with @ref NarrowedNumericFormat to have a smaller
///                          representation in coded form.
template <typename T, typename UnderlyingFormat = Format<typename T::value_type>>
struct NamedTypeFormat {
  static constexpr size_t encodedSize(FileVersion const& version, T const& value) {
    return UnderlyingFormat::encodedSize(version, value.value());
  }

  static size_t encode(EncodeTarget target, FileVersion const& version, T const& value) {
    return UnderlyingFormat::encode(target, version, value.value());
  }

  static size_t decode(T& output, FileVersion const& version, DecodeSource source) {
    return UnderlyingFormat::decode(output.value(), version, source);
  }
};

/// A formatter for fixed-size arrays.
/// Elements are stored end-to-end.
/// The number of elements is not encoded.
///
/// @tparam TArray The array type to expect to encode from or decode into.
///                All functions require a member function named @c size and @c data
///                for element access. If the underlying type is dynamically-sized, the
///                decoder assumes it has already been resized properly.
/// @tparam ElementFormat The format of the elements in the array.
template <typename TArray, typename ElementFormat = Format<typename TArray::value_type>>
struct FixedSizeArrayFormat {
  template <typename UArray>
  static constexpr size_t encodedSize(FileVersion const& version, UArray&& values) {
    size_t totalSz = 0U;
    for (size_t idx = 0U; idx < values.size(); ++idx) {
      totalSz += ElementFormat::encodedSize(version, values.data()[idx]);
    }
    return totalSz;
  }

  template <typename UArray>
  static size_t encode(EncodeTarget target, FileVersion const& version, UArray&& values) {
    size_t encodedSz = 0U;
    for (size_t idx = 0U; idx < values.size(); ++idx) {
      auto elemSpan = target.subspan(encodedSz).value();
      encodedSz += ElementFormat::encode(elemSpan, version, values.data()[idx]);
    }
    return encodedSz;
  }

  template <typename UArray>
  static size_t decode(UArray&& output, FileVersion const& version, DecodeSource source) {
    size_t decodedSz = 0U;
    for (size_t idx = 0U; idx < output.size(); ++idx) {
      auto rest = source.subspan(decodedSz).value();
      decodedSz += ElementFormat::decode(output.data()[idx], version, rest);
    }
    return decodedSz;
  }
};

template <typename T, size_t N>
struct Format<T[N]> {
  using SpanType      = Span<T>;
  using ConstSpanType = Span<T const>;

  static constexpr size_t encodedSize(FileVersion const& version, T const (&value)[N]) {
    return FixedSizeArrayFormat<SpanType>::encodedSize(version, ConstSpanType(value, N));
  }

  static size_t encode(EncodeTarget target, FileVersion const& version, T const (&value)[N]) {
    return FixedSizeArrayFormat<SpanType>::encode(target, version, ConstSpanType(value, N));
  }

  static size_t decode(T (&output)[N], FileVersion const& version, DecodeSource source) {
    return FixedSizeArrayFormat<SpanType>::decode(SpanType(output, N), version, source);
  }
};

template <typename T, size_t N>
struct Format<std::array<T, N>> : FixedSizeArrayFormat<std::array<T, N>> {};

/// A formatter for dynamically-sized arrays.
/// The number of elements is recorded first, then elements are stored
/// end-to-end in the same manner as @ref FixedSizeArrayFormat.
///
/// @tparam TArray The array type to expect to encode from or decode to.
///                This has the same requirements as @c TArray for @ref FixedSizeArrayFormat
///                and additionally requires a @c resize function for decoding.
/// @tparam ElementFormat The format of the elements in the array.
/// @tparam SizeFormat The format to use for recording the number of elements in the array.
template <typename TArray,
          typename ElementFormat = Format<typename TArray::value_type>,
          typename SizeFormat    = NarrowedNumericFormat<typename TArray::size_type, uint16_t>>
struct DynamicSizeArrayFormat {
  using ArrayPortionFormat = FixedSizeArrayFormat<TArray, ElementFormat>;

  static size_t encodedSize(FileVersion const& version, TArray const& value) {
    auto sizeSz = SizeFormat::encodedSize(version, value.size());
    return sizeSz + ArrayPortionFormat::encodedSize(version, value);
  }

  static size_t encode(EncodeTarget target, FileVersion const& version, TArray const& value) {
    auto sizeSz  = SizeFormat::encode(target, version, value.size());
    auto arraySz = ArrayPortionFormat::encode(target.subspan(sizeSz).value(), version, value);
    return sizeSz + arraySz;
  }

  static size_t decode(TArray& output, FileVersion const& version, DecodeSource source) {
    typename TArray::size_type outputCount{};
    auto sizeSz = SizeFormat::decode(outputCount, version, source);
    output.resize(outputCount);
    auto arraySz = ArrayPortionFormat::decode(output, version, source.subspan(sizeSz).value());
    return sizeSz + arraySz;
  }
};

/// Similar to @ref DynamicSizeArrayFormat, but the contents are not copied from the
/// source on decode. Rather, a view of the elements is returned in the output.
/// Since no element decoding can happen, no element encoding can happen.
template <typename T, typename SizeFormat = NarrowedNumericFormat<size_t, uint16_t>>
struct DynamicSizeViewFormat {
  static_assert(sizeof(T) == 1U, "This format only works when the element size is 1");

  static size_t encodedSize(FileVersion const& version, Span<T const> const& value) {
    auto sizeSz = SizeFormat::encodedSize(version, value.size());
    return sizeSz + value.size();
  }

  static size_t encode(EncodeTarget target,
                       FileVersion const& version,
                       Span<T const> const& value) {
    auto sizeSz = SizeFormat::encode(target, version, value.size());
    std::memcpy(static_cast<void*>(target.subspan(sizeSz, value.size())->data()),
                static_cast<void const*>(value.data()),
                value.size());
    return sizeSz + value.size();
  }

  static size_t decode(Span<T const>& output, FileVersion const& version, DecodeSource source) {
    typename Span<T const>::size_type outputCount{};

    auto sizeSz     = SizeFormat::decode(outputCount, version, source);
    auto outputSpan = source.subspan(sizeSz, outputCount).value();

    output = Span<T const>(reinterpret_cast<T const*>(outputSpan.data()), outputCount);
    return sizeSz + outputCount;
  }
};

/// @warning
/// Unlike most format implementations, the @ref encodedSize of this unwritten segment is
/// different from the value returned from @ref encode and @ref decode. It is the responsibility
/// of higher-level constructs to understand that a @ref DynamicUnwritten type (which this encodes)
/// is unwritten.
///
/// @see TrackHeaderFrame
/// @see DataFrame
template <typename SizeFormat = Format<uint64_t>>
struct DynamicUnwrittenFormat {
  static size_t encodedSize(FileVersion const& version, DynamicUnwritten const& block) {
    return SizeFormat::encodedSize(version, block.size) + block.size;
  }

  static size_t encode(EncodeTarget target,
                       FileVersion const& version,
                       DynamicUnwritten const& block) {
    return SizeFormat::encode(target, version, block.size);
  }

  static size_t decode(DynamicUnwritten& output, FileVersion const& version, DecodeSource source) {
    return SizeFormat::decode(output.size, version, source);
  }
};

/// @see ComposedFormats
template <typename TWhole,
          typename TMember,
          TMember TWhole::*Select,
          typename TMemberFormat = Format<TMember>>
struct FormatMember {
  using MemberFormat = TMemberFormat;

  static TMember& select(TWhole& src) {
    return src.*Select;
  }

  static TMember const& select(TWhole const& src) {
    return src.*Select;
  }
};

/// Seeing compiler errors with messages about "invalid use of incomplete type" for
/// @c ComposedFormats? This will happen if @a TMembers is not an @c std::tuple of
/// @ref FormatMember types. If it looks like everything is already a list of
/// @ref FormatMember types in your source, look really closely at the list at the
/// end of the struct description. There is probably a tiny error or mismatch in the list.
/// Messages containing things like "overloaded function" probably point to the problem.
template <typename TWhole, typename TMembers>
struct ComposedFormats;

template <typename TWhole, typename... TMember, TMember TWhole::*... Select, typename... TFormat>
struct ComposedFormats<TWhole, std::tuple<FormatMember<TWhole, TMember, Select, TFormat>...>> {
  static constexpr size_t encodedSize(FileVersion const& version, TWhole const& value) {
    return encodedSizeImpl<0U>(version, value, 0U);
  }

  static size_t encode(EncodeTarget target, FileVersion const& version, TWhole const& value) {
    return encodeImpl<0U>(target, 0U, version, value);
  }

  static size_t decode(TWhole& output, FileVersion const& version, DecodeSource source) {
    return decodeImpl<0U>(output, version, source, 0U);
  }

 private:
  using Members = std::tuple<FormatMember<TWhole, TMember, Select, TFormat>...>;

  static constexpr size_t Size = std::tuple_size<Members>::value;

  template <size_t Idx>
  using MemberAt = std::decay_t<decltype(std::get<Idx>(std::declval<Members>()))>;

  template <size_t Idx>
  static constexpr size_t encodedSizeSingle(FileVersion const& version, TWhole const& value) {
    using Member = MemberAt<Idx>;
    return Member::MemberFormat::encodedSize(version, Member::select(value));
  }

  template <size_t Idx>
  static constexpr EnableIf_t<(Idx == Size), size_t>
  encodedSizeImpl(FileVersion const&, TWhole const&, size_t currentSize) {
    return currentSize;
  }

  template <size_t Idx>
  static constexpr EnableIf_t<(Idx < Size), size_t>
  encodedSizeImpl(FileVersion const& version, TWhole const& value, size_t currentSize) {
    return encodedSizeImpl<Idx + 1U>(version,
                                     value,
                                     currentSize + encodedSizeSingle<Idx>(version, value));
  }

  template <size_t Idx>
  static size_t encodeSingle(EncodeTarget target, FileVersion const& version, TWhole const& value) {
    using Member = MemberAt<Idx>;
    return Member::MemberFormat::encode(target, version, Member::select(value));
  }

  template <size_t Idx>
  static EnableIf_t<(Idx == Size), size_t>
  encodeImpl(EncodeTarget, size_t currentOffset, FileVersion const&, TWhole const&) {
    return currentOffset;
  }

  template <size_t Idx>
  static EnableIf_t<(Idx < Size), size_t>
  encodeImpl(EncodeTarget target,
             size_t currentOffset,
             FileVersion const& version,
             TWhole const& value) {
    auto encodedSz = encodeSingle<Idx>(target, version, value);

    target = target.subspan(encodedSz).value();
    return encodeImpl<Idx + 1U>(target, currentOffset + encodedSz, version, value);
  }

  template <size_t Idx>
  static size_t decodeSingle(TWhole& output, FileVersion const& version, DecodeSource source) {
    using Member = MemberAt<Idx>;
    return Member::MemberFormat::decode(Member::select(output), version, source);
  }

  template <size_t Idx>
  static EnableIf_t<(Idx == Size), size_t>
  decodeImpl(TWhole&, FileVersion const&, DecodeSource, size_t currentOffset) {
    return currentOffset;
  }

  template <size_t Idx>
  static EnableIf_t<(Idx < Size), size_t>
  decodeImpl(TWhole& output,
             FileVersion const& version,
             DecodeSource source,
             size_t currentOffset) {
    auto decodedSz = decodeSingle<Idx>(output, version, source);

    source = source.subspan(decodedSz).value();
    return decodeImpl<Idx + 1U>(output, version, source, currentOffset + decodedSz);
  }
};

/// @def POD_FORMAT_MEMBER
/// Create a @ref FormatMember type for the given @a type_name_ and @a member_name_.
/// For example, if you wanted to format the @c key field of @c MyType, you would say
/// `POD_FORMAT_MEMBER(MyType, key)`. This is used for building the members tuple of a
/// @ref ComposedFormats -- see there for example usage.
///
/// @note
/// The information we need is held completely by the expression `&type_name_::member_name_`,
/// but there is no way to capture that as a template parameter until C++17. With C++17,
/// this macro can be removed and @c FormatMember can be used directly with a generic
/// non-type template parameter placeholder @a auto.
///
/// @code
/// template <auto Select, typename TFormat>
/// struct FormatMember;
///
/// template <typename TWhole, typename TMember, TMember TWhole::*Select, typename TFormat>
/// struct FormatMember<Select, TFormat>
/// { ... };
/// @endcode
#define POD_FORMAT_MEMBER(type_name_, member_name_)    \
    nvidia::gxf::pod::FormatMember<                    \
    type_name_,                                        \
    decltype(std::declval<type_name_>().member_name_), \
    &type_name_::member_name_>

////////////////////////////////////////////////////////////////////////////////////////////////////
// Specific Formats                                                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

template <>
struct Format<Position> : NamedTypeFormat<Position> {};

template <>
struct Format<BundleId> : NamedTypeFormat<BundleId> {};

template <>
struct Format<SearchStamp> : NamedTypeFormat<SearchStamp> {};

template <>
struct Format<TrackId> :
  NamedTypeFormat
  <
    TrackId,
    NarrowedNumericFormat
    <
      TrackId::value_type,
      uint16_t
    >
  >
{};

template <>
struct Format<DataCompressionMode> {
  using Representation = std::array<char, 4U>;

  using EncodedFormat = Format<Representation>;

  static size_t encodedSize(FileVersion const& version, DataCompressionMode const& value);

  static size_t encode(EncodeTarget target,
                       FileVersion const& version,
                       DataCompressionMode const& value);

  static size_t decode(DataCompressionMode& output,
                       FileVersion const& version,
                       DecodeSource source);
};

template <>
struct Format<Span<char const>> : DynamicSizeViewFormat<char> {};

template <>
struct Format<Span<uint8_t const>> : DynamicSizeViewFormat<uint8_t> {};

template <>
struct Format<Span<SearchStampType const>> : DynamicSizeViewFormat<SearchStampType> {};

template <>
struct Format<SearchStampList> : DynamicSizeArrayFormat<SearchStampList> {};

template <>
struct Format<FileVersion> :
  ComposedFormats
  <
    FileVersion,
    std::tuple
    <
      POD_FORMAT_MEMBER(FileVersion, major),
      POD_FORMAT_MEMBER(FileVersion, minor)
    >
  >
{};

template <>
struct Format<EncodedFieldView> :
  ComposedFormats
  <
    EncodedFieldView,
    std::tuple
    <
      POD_FORMAT_MEMBER(EncodedFieldView, name),
      POD_FORMAT_MEMBER(EncodedFieldView, value)
    >
  >
{};

template <>
struct Format<FieldList> :
  WrapSizeFormat
  <
    DynamicSizeArrayFormat<FieldList>
  >
{};

template <>
struct Format<SearchStampRangeEntry> :
  ComposedFormats
  <
    SearchStampRangeEntry,
    std::tuple
    <
      POD_FORMAT_MEMBER(SearchStampRangeEntry, a),
      POD_FORMAT_MEMBER(SearchStampRangeEntry, b)
    >
  >
{};

template <>
struct Format<SearchStampRangeList> :
  WrapSizeFormat
  <
    DynamicSizeArrayFormat
    <
      SearchStampRangeList
    >
  >
{};

template <>
struct Format<TrackKnownInfo> :
  ComposedFormats
  <
    TrackKnownInfo,
    std::tuple
    <
      POD_FORMAT_MEMBER(TrackKnownInfo, trackId),
      POD_FORMAT_MEMBER(TrackKnownInfo, trackHeaderOffset),
      POD_FORMAT_MEMBER(TrackKnownInfo, lastDataFrameOffset),
      POD_FORMAT_MEMBER(TrackKnownInfo, lastSeekFrameOffset)
    >
  >
{};

template <>
struct Format<TrackKnownInfoList> : WrapSizeFormat<DynamicSizeArrayFormat<TrackKnownInfoList>> {};

template <>
struct Format<FooterTrackKnownInfo> :
  ComposedFormats
  <
    FooterTrackKnownInfo,
    std::tuple
    <
      POD_FORMAT_MEMBER(FooterTrackKnownInfo, trackId),
      POD_FORMAT_MEMBER(FooterTrackKnownInfo, trackHeaderOffset),
      POD_FORMAT_MEMBER(FooterTrackKnownInfo, lastDataFrameOffset),
      POD_FORMAT_MEMBER(FooterTrackKnownInfo, lastSeekFrameOffset),
      POD_FORMAT_MEMBER(FooterTrackKnownInfo, indexFrameOffset)
    >
  >
{};

template <>
struct Format<FooterTrackKnownInfoList> :
  WrapSizeFormat
  <
    DynamicSizeArrayFormat
    <
      FooterTrackKnownInfoList
    >
  >
{};


template <>
struct Format<FrameIntro> :
  ComposedFormats
  <
    FrameIntro,
    std::tuple
    <
      POD_FORMAT_MEMBER(FrameIntro, frameSize),
      POD_FORMAT_MEMBER(FrameIntro, frameType)
    >
  >
{};

template <>
struct Format<FileHeader> :
  ComposedFormats
  <
    FileHeader,
    std::tuple
    <
      POD_FORMAT_MEMBER(FileHeader, fileMagicNumber),
      POD_FORMAT_MEMBER(FileHeader, fileVersion),
      POD_FORMAT_MEMBER(FileHeader, driveworksMajor),
      POD_FORMAT_MEMBER(FileHeader, driveworksMinor),
      POD_FORMAT_MEMBER(FileHeader, driveworksPatch),
      POD_FORMAT_MEMBER(FileHeader, driveworksHash)
    >
  >
{};

template <>
struct Format<PodHeaderFrame> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      PodHeaderFrame,
      std::tuple
      <
        POD_FORMAT_MEMBER(PodHeaderFrame, frameType),
        POD_FORMAT_MEMBER(PodHeaderFrame, formatVersion),
        POD_FORMAT_MEMBER(PodHeaderFrame, checkpointInterval),
        FormatMember
        <
          PodHeaderFrame,
          FixedVector<Position, TRACK_COUNT_MAX>,
          &PodHeaderFrame::trackHeaderFrameOffsets,  // NOLINT
          DynamicSizeArrayFormat<FixedVector<Position, TRACK_COUNT_MAX>>
        >
      >
    >
  >
{};

template <>
struct Format<TrackHeaderFrameHeader> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      TrackHeaderFrameHeader,
      std::tuple
      <
        POD_FORMAT_MEMBER(TrackHeaderFrameHeader, trackId),
        POD_FORMAT_MEMBER(TrackHeaderFrameHeader, dataCompressionMode),
        POD_FORMAT_MEMBER(TrackHeaderFrameHeader, searchStampTypes),
        POD_FORMAT_MEMBER(TrackHeaderFrameHeader, fields)
      >
    >
  >
{};

template <>
struct Format<TrackHeaderFrame> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      TrackHeaderFrame,
      std::tuple
      <
        POD_FORMAT_MEMBER(TrackHeaderFrame, frameType),
        POD_FORMAT_MEMBER(TrackHeaderFrame, header),
        FormatMember
        <
          TrackHeaderFrame,
          DynamicUnwritten,
          &TrackHeaderFrame::unstructuredData,  // NOLINT
          DynamicUnwrittenFormat<>
        >
      >
    >,
    Format<uint64_t>,
    0U,
    false
  >
{};

template <>
struct Format<DataFrameHeader> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      DataFrameHeader,
      std::tuple
      <
        POD_FORMAT_MEMBER(DataFrameHeader, trackId),
        POD_FORMAT_MEMBER(DataFrameHeader, bundleId),
        POD_FORMAT_MEMBER(DataFrameHeader, searchStamps),
        POD_FORMAT_MEMBER(DataFrameHeader, fields)
      >
    >
  >
{};

template <>
struct Format<DataFrame> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      DataFrame,
      std::tuple
      <
        POD_FORMAT_MEMBER(DataFrame, frameType),
        POD_FORMAT_MEMBER(DataFrame, header),
        FormatMember
        <
          DataFrame,
          DynamicUnwritten,
          &DataFrame::data,  // NOLINT
          DynamicUnwrittenFormat<>
        >
      >
    >,
    Format<uint64_t>,
    0U,
    false
  >
{};

template <>
struct Format<SeekFrame> :
  WrapSizeFormat
  <
    ComposedFormats
    <
      SeekFrame,
      std::tuple
      <
        POD_FORMAT_MEMBER(SeekFrame, frameType),
        POD_FORMAT_MEMBER(SeekFrame, trackId),
        POD_FORMAT_MEMBER(SeekFrame, firstDataFrameOffset),
        POD_FORMAT_MEMBER(SeekFrame, lastDataFrameOffset),
        POD_FORMAT_MEMBER(SeekFrame, previousSeekFrameOffset),
        POD_FORMAT_MEMBER(SeekFrame, nextSeekFrameOffset),
        POD_FORMAT_MEMBER(SeekFrame, contained)
      >
    >,
    Format<uint64_t>
  >
{};

template <>
struct Format<FooterTail> :
  ComposedFormats
  <
    FooterTail,
    std::tuple
    <
      POD_FORMAT_MEMBER(FooterTail, frameSize),
      POD_FORMAT_MEMBER(FooterTail, digest)
    >
  >
{};

template <>
struct Format<FooterFrame> {
  static size_t encodedSize(FileVersion const& version, FooterFrame const& value);

  static size_t encode(EncodeTarget target, FileVersion const& version, FooterFrame const& value);

  static size_t decode(FooterFrame& output, FileVersion const& version, DecodeSource source);

  struct InternalFormat;
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_FORMATS_HPP_
