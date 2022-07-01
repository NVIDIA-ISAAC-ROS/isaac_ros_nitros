/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_HEADER_HPP_
#define NVIDIA_GXF_DW_POD_HEADER_HPP_

#include <string>
#include <utility>

#include "common/optional.hpp"
#include "common/span.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/dw/pod/types.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// @note
/// While the underlying format has more frame types than @c kTrackHeadere and @c kData,
/// these are not exposed externally.
enum class FrameType : uint8_t {
  /// An unknown frame was encountered, which is expected if file was encoded with a future
  /// version of the format, but can also happen if the data is corrupt. Check the logs for
  /// more information.
  UNKNOWN,
  /// A track header frame introduces a new track.
  TRACK_HEADER,
  /// A data frame contains the next fragment of data for a given track.
  DATA,
};

/// A view of a header from a pod.
///
/// @warning
/// This is only a view of the contents of a header and is backed by the internal buffer of
/// whatever gave you the instance. This means the references could be invalidated by an operation
/// on their source. This includes copies of this structure, even if you put those copies into a
/// @c shared_ptr. Any function which returns a @c HeaderView should come with documentation on
/// when the returned value is invalidated.
///
/// @see Reader::currentHeader
struct HeaderView {
  /// The position this frame starts at.
  Position position{};

  /// The total size of this frame, including the contents of the data following the header.
  size_t frameSize{};

  /// The ID of the track this is the frame header for.
  TrackId trackId{};

  /// The type of frame this is the frame header for.
  FrameType frameType{};

  /// The bundle this frame is associated with.
  /// If @ref frameType is @c kTrackHeader, this will be @c NULLOPT.
  std::optional<BundleId> bundleId{};

  /// The list of fields.
  ///
  /// @see findField
  Span<EncodedFieldView const> fields{};

  /// The types associated with the @a searchStamps list. Note that this field is populated
  /// even when @ref frameType is @c kData.
  Span<SearchStampType const> searchStampTypes{};

  /// The search stamps for this @c kData frame.
  /// If @ref frameType is @c kTrackHeader, this will be empty.
  Span<SearchStamp const> searchStamps{};

  /// The size of the data. The always refers to the size of the uncompressed data -- if data
  /// compression was enabled, for this track @c dataSize will not simply be the @c data field
  /// of the underlying format.
  uint64_t dataSize{};

  HeaderView() = default;

  HeaderView(Position positionIn,
             size_t frameSizeIn,
             TrackId trackIdIn,
             FrameType frameTypeIn,
             std::optional<BundleId> bundleIdIn,
             Span<EncodedFieldView const> fieldsIn,
             Span<SearchStampType const> searchStampTypesIn,
             Span<SearchStamp const> searchStampsIn,
             uint64_t dataSizeIn) noexcept
      : position(positionIn),
        frameSize(frameSizeIn),
        trackId(trackIdIn),
        frameType(frameTypeIn),
        bundleId(std::move(bundleIdIn)),
        fields(std::move(fieldsIn)),
        searchStampTypes(std::move(searchStampTypesIn)),
        searchStamps(std::move(searchStampsIn)),
        dataSize(dataSizeIn) {}

  /// @{
  /// Attempt to find the field with the given @a name.
  ///
  /// @return the field if with the given @a name or an `Unexpected` if it does not exist.
  Expected<EncodedFieldView> findField(Span<char const> name) const;
  Expected<EncodedFieldView> findField(char const* name) const;
  Expected<EncodedFieldView> findField(std::string const& name) const;
  /// @}
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_HEADER_HPP_
