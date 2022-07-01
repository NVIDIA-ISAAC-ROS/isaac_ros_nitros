/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_TYPES_HPP_
#define NVIDIA_GXF_DW_POD_TYPES_HPP_

#include <limits>
#include <utility>

#include "common/span.hpp"
#include "common/strong_type.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

struct TrackId : StrongType<TrackId, uint64_t> {
  using StrongType<TrackId, uint64_t>::StrongType;
};

static constexpr TrackId TRACK_ID_NONE = TrackId(0U);

/// The maxium supported number of tracks.
constexpr size_t TRACK_COUNT_MAX = 32U;

/// An identifier used to group otherwise-unrelated frames together
/// for the purposes of presentation.
struct BundleId : StrongType<BundleId, uint64_t> {
  using StrongType<BundleId, uint64_t>::StrongType;
};

/// No bundle.
static constexpr BundleId BUNDLE_ID_NONE = BundleId(0U);

struct SearchStamp : StrongType<SearchStamp, int64_t> {
  using StrongType<SearchStamp, int64_t>::StrongType;
};

/// This value determines how @ref SearchStamp values are interpreted for building seek information.
enum class SearchStampType : uint8_t {
  /// The search stamp does not have any known interpretation. This should not be used when
  /// creating a track, but it is possible to see this value when reading if the file has a
  /// value which is unknown (perhaps it was written in a future version of the Pod format).
  NONE = 0,
  /// The @ref SearchStamp values should be interpreted as a range. This is useful for things
  /// like timestamps, where queries take the form of "I wish to seek to 24 minutes into the
  /// track" or "find frame 4023."
  RANGE = 1,
  /// The @ref SearchStamp values should be interpreted as bit flags. This is useful for things
  /// like events, where queries take the form of "I want to find a place where a stop sign was
  /// seen."
  FLAGS = 2,
};

Span<SearchStampType const> getAllSearchStampTypes();

/// The maximum supported number of search stamps.
constexpr size_t SEARCH_STAMP_MAX = 128U;

/// @struct Position
/// Refers to an exact position in the file.
///
/// @see POSITION_NONE
struct Position : StrongType<Position, uint64_t> {
  using StrongType<Position, uint64_t>::StrongType;

  /// Move this position forward by @a count bytes.
  constexpr Position& operator+=(size_t count) noexcept {
    value() += count;
    return *this;
  }

  /// Return a position @a count bytes ahead of this one.
  constexpr Position operator+(size_t count) const noexcept {
    Position position = *this;
    position += count;
    return position;
  }

  /// Calculate the distance in bytes between this and @a other.
  constexpr ptrdiff_t operator-(Position other) const noexcept {
    return ptrdiff_t(this->value()) - ptrdiff_t(other.value());
  }
};

/// Represents the zero position.
constexpr Position POSITION_NONE{0U};

enum class DataCompressionMode : uint8_t {
  NONE = 0x00,
  /// An unknown compression mode.
  UNKNOWN = std::numeric_limits<uint8_t>::max(),
};

Span<DataCompressionMode const> getAllDataCompressionModes();

// Splitting in struct and global functions to ensure sizeof(FileHeader) is sane
static constexpr size_t HASH_LENGTH = 12;
struct FileHeader {
  uint32_t fileMagicNumber;
  int32_t fileVersion;
  int32_t driveworksMajor;
  int32_t driveworksMinor;
  int32_t driveworksPatch;
  int8_t driveworksHash[HASH_LENGTH];
  // 32 bytes total
} __attribute__((packed, aligned(1)));

FileHeader createFileHeader(uint32_t magicNumber, int32_t version);

struct FileVersion {
  uint16_t major;
  uint16_t minor;

  static FileVersion getCurrent();

  FileVersion() = default;

  constexpr FileVersion(uint16_t majorIn, uint16_t minorIn) : major{majorIn}, minor{minorIn} {}

  /// Create an instance from an @a encoded value.
  /// This can come from @ref encoded or from @ref FileHeader::fileVersion.
  static constexpr FileVersion fromCoded(int32_t encoded) {
    return {uint16_t(uint32_t(encoded) >> 16), uint16_t(encoded)};
  }

  /// Convert this instance into the type used in @ref FileHeader::encoded.
  constexpr int32_t encoded() const { return (int32_t(major) << 16) | int32_t(minor); }
};

inline constexpr bool operator==(FileVersion const& lhs, FileVersion const& rhs) {
  return lhs.major == rhs.major && lhs.minor == rhs.minor;
}

inline constexpr bool operator!=(FileVersion const& lhs, FileVersion const& rhs) {
  return !(lhs == rhs);
}

inline constexpr bool operator<(FileVersion const& lhs, FileVersion const& rhs) {
  return lhs.encoded() < rhs.encoded();
}

inline constexpr bool operator>(FileVersion const& lhs, FileVersion const& rhs) {
  return rhs < lhs;
}

inline constexpr bool operator<=(FileVersion const& lhs, FileVersion const& rhs) {
  return !(rhs < lhs);
}

inline constexpr bool operator>=(FileVersion const& lhs, FileVersion const& rhs) {
  return !(lhs < rhs);
}

/// An encoded view of a field.
struct EncodedFieldView final {
  /// The name of this field.
  Span<char const> name;
  /// The encoded bytes of the value for this field.
  Span<uint8_t const> value;

  EncodedFieldView() = default;

  EncodedFieldView(Span<char const> nameIn, Span<uint8_t const> valueIn) noexcept
      : name(std::move(nameIn)), value(std::move(valueIn)) {}
};

bool operator==(EncodedFieldView const& lhs, EncodedFieldView const& rhs);
bool operator!=(EncodedFieldView const& lhs, EncodedFieldView const& rhs);

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_TYPES_HPP_
