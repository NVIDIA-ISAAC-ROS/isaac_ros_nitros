/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_DATA_HPP_
#define NVIDIA_GXF_DW_POD_DATA_HPP_

#include <array>
#include <utility>

#include "common/fixed_vector.hpp"
#include "common/span.hpp"
#include "gxf/dw/pod/types.hpp"

/// @file
/// This file contains structure definitions of on-disk contents of DW Pod. Each structure has
/// a corresponding format descriptor (by way of a @c Format<T> specialization) in the
/// @c formats.hpp sibling to this. The full documentation for these structures can be found in the
/// DriveWorks Pod documentation under the "Types" header. These data types are meant to be simple
/// passthrough representations. Higher-level things like semantic validity checks are managed by
/// the @c Reader and @c Writer implementations while lower-level things like encoded format
/// validity checks are managed by @c Format<T>.

namespace nvidia {
namespace gxf {
namespace pod {

// FIXME: Pick a real magic number here.
// The current number is "POD\\n" in ASCII when laid out on disk.
static constexpr uint32_t FILE_MAGIC_NUMBER = 0x0a444f50U;

/// Contains references to information about a track known at a certain point in time.
struct TrackKnownInfo {
  /// The ID of the track.
  TrackId trackId;
  /// The offset of the `TrackHeaderFrame` for this track.
  /// Since header frames do not move after they are written, this is always authoritative.
  Position trackHeaderOffset;
  /// The offset of the last written `DataFrame` for this track.
  /// The value `0` indicates that no data has been written for this track.
  Position lastDataFrameOffset;
  /// The offset of the last written `SeekFrame` for this track.
  /// The value `0` indicates that no seek frames have been written for this track.
  Position lastSeekFrameOffset;

  explicit TrackKnownInfo() = default;

  explicit TrackKnownInfo(TrackId trackIdIn,
                          Position trackHeaderOffsetIn,
                          Position lastDataFrameOffsetIn,
                          Position lastSeekFrameOffsetIn)
      : trackId(trackIdIn),
        trackHeaderOffset(trackHeaderOffsetIn),
        lastDataFrameOffset(lastDataFrameOffsetIn),
        lastSeekFrameOffset(lastSeekFrameOffsetIn) {}

  friend bool operator==(TrackKnownInfo const&, TrackKnownInfo const&);
  friend bool operator!=(TrackKnownInfo const&, TrackKnownInfo const&);
};

using TrackKnownInfoList = FixedVector<TrackKnownInfo, TRACK_COUNT_MAX>;

/// Similar to @c TrackKnownInfo, but only used in the footer.
struct FooterTrackKnownInfo {
  /// The ID of the track.
  TrackId trackId;
  /// The offset of the `TrackHeaderFrame` for this track.
  /// Since header frames do not move after they are written, this is always authoritative.
  Position trackHeaderOffset;
  /// The offset of the last written `DataFrame` for this track.
  /// The value `0` indicates that no data has been written for this track.
  Position lastDataFrameOffset;
  /// The offset of the last written `SeekFrame` for this track.
  /// The value `0` indicates that no seek frames have been written for this track.
  Position lastSeekFrameOffset;
  /// The offset of the `IndexFrame` for this track.
  /// The value `0` indicates that no index frame has been written for this track.
  Position indexFrameOffset;

  explicit FooterTrackKnownInfo() = default;
  explicit FooterTrackKnownInfo(TrackId trackIdIn,
                                Position trackHeaderOffsetIn,
                                Position lastDataFrameOffsetIn,
                                Position lastSeekFrameOffsetIn,
                                Position indexFrameOffsetIn)
      : trackId(trackIdIn),
        trackHeaderOffset(trackHeaderOffsetIn),
        lastDataFrameOffset(lastDataFrameOffsetIn),
        lastSeekFrameOffset(lastSeekFrameOffsetIn),
        indexFrameOffset(indexFrameOffsetIn) {}

  friend bool operator==(FooterTrackKnownInfo const&, FooterTrackKnownInfo const&) noexcept;
  friend bool operator!=(FooterTrackKnownInfo const&, FooterTrackKnownInfo const&) noexcept;
};

using FooterTrackKnownInfoList = FixedVector<FooterTrackKnownInfo, TRACK_COUNT_MAX>;

using FrameTypeIndicator = std::array<char, 8U>;

extern FrameTypeIndicator const FRAME_TYPE_NONE;

using FieldList = FixedVector<EncodedFieldView, 128U>;

using SearchStampList = FixedVector<SearchStamp, SEARCH_STAMP_MAX>;

/// A dynamically-sized field that is not written, but still contributes to the encoded size.
/// This is used for the unstructured data field that follows @ref DataFrame and
/// @ref TrackHeaderFrame.
struct DynamicUnwritten {
  size_t size{0U};

  explicit DynamicUnwritten() = default;
  explicit DynamicUnwritten(size_t sizeIn) : size(sizeIn) {}
};

/// Introduces a frame.
struct FrameIntro {
  uint64_t frameSize;
  FrameTypeIndicator frameType;

  FrameIntro() = default;
  FrameIntro(uint64_t frameSizeIn, FrameTypeIndicator const& frameTypeIn)
      : frameSize(frameSizeIn), frameType(frameTypeIn) {}
};

struct PodHeaderFrame {
  static FrameTypeIndicator const FRAME_TYPE;

  FrameTypeIndicator frameType{FRAME_TYPE};
  /// The version of encoding to expect in this file.
  /// This will always be the same as `FileHeader::fileVersion`.
  FileVersion formatVersion{};
  /// The interval at which `CheckpointFrames` should be written.
  /// The default value for this is 1GB, but this can be altered through `Writer` settings.
  /// If this number is `0`, checkpointing is disabled.
  uint64_t checkpointInterval{};
  FixedVector<Position, TRACK_COUNT_MAX> trackHeaderFrameOffsets{};

  PodHeaderFrame() = default;

  explicit PodHeaderFrame(FileVersion formatVersionIn, uint64_t checkpointIntervalIn)
      : formatVersion(formatVersionIn),
        checkpointInterval(checkpointIntervalIn),
        // We don't know track header positions yet
        trackHeaderFrameOffsets() {}

  friend bool operator==(PodHeaderFrame const&, PodHeaderFrame const&);
  friend bool operator!=(PodHeaderFrame const&, PodHeaderFrame const&);
};

struct TrackHeaderFrameHeader {
  /// The ID of the track this header describes.
  TrackId trackId{};
  /// The compression mode to be applied to data frames.
  /// If compression is enabled, the `data` of the header will also be
  /// compressed with the same algorithm.
  DataCompressionMode dataCompressionMode{};
  /// A list of types the [`DataFrameHeader::searchStamps`].
  /// The values in this list determine the meaning of the values in `SearchStampRangeEntry`.
  Span<SearchStampType const> searchStampTypes{};
  /// Arbitrary, user-defined key-value pairs for this frame which are not indexed.
  FieldList fields{};

  explicit TrackHeaderFrameHeader() = default;

  explicit TrackHeaderFrameHeader(TrackId trackIdIn,
                                  DataCompressionMode dataCompressionModeIn,
                                  Span<SearchStampType const> searchStampTypesIn,
                                  Span<EncodedFieldView const> fieldsIn)
      : trackId(trackIdIn),
        dataCompressionMode(dataCompressionModeIn),
        searchStampTypes(std::move(searchStampTypesIn)),
        fields() {
    for (auto const& field : fieldsIn) {
      fields.emplace_back(field.value());
    }
  }

  friend bool operator==(TrackHeaderFrameHeader const&, TrackHeaderFrameHeader const&);
  friend bool operator!=(TrackHeaderFrameHeader const&, TrackHeaderFrameHeader const&);
};

struct TrackHeaderFrame {
  static FrameTypeIndicator const FRAME_TYPE;

  FrameTypeIndicator frameType = FRAME_TYPE;
  TrackHeaderFrameHeader header{};
  /// Generic data for this header.
  /// This is used for codecs which have a pre-existing binary format for their header,
  /// such as H.264.
  DynamicUnwritten unstructuredData{};

  explicit TrackHeaderFrame() = default;

  explicit TrackHeaderFrame(TrackId trackIdIn,
                            DataCompressionMode dataCompressionModeIn,
                            Span<SearchStampType const> searchStampTypesIn,
                            Span<EncodedFieldView const> fieldsIn,
                            size_t unstructuredDataSize)
      : header(trackIdIn, dataCompressionModeIn, searchStampTypesIn, fieldsIn),
        unstructuredData(unstructuredDataSize) {}

  friend bool operator==(TrackHeaderFrame const&, TrackHeaderFrame const&);
  friend bool operator!=(TrackHeaderFrame const&, TrackHeaderFrame const&);
};

struct DataFrameHeader {
  /// The ID of the track this data is for.
  TrackId trackId{};
  /// The bundle to group frames together for the purposes of presentation.
  /// The special value `0` means there is no associated bundle for this frame.
  /// It is recommended that bundles are monotonically increasing, but this is not required.
  BundleId bundleId{};
  /// The list of stamps used for searching.
  /// The meaning of these values is described in the `TrackHeaderFrame::searchStampTypes`
  /// for this trackId, which will always be the same size as this list.
  SearchStampList searchStamps{};
  /// Arbitrary, user-defined key-value pairs for this frame which are not indexed.
  FieldList fields{};

  explicit DataFrameHeader() = default;
  explicit DataFrameHeader(TrackId trackIdIn,
                           BundleId bundleIdIn,
                           Span<SearchStamp const> searchStampsIn,
                           Span<EncodedFieldView const> fieldsIn)
      : trackId(trackIdIn),
        bundleId(bundleIdIn),
        searchStamps(),
        fields() {
    for (auto const& searchStamp : searchStampsIn) {
      searchStamps.emplace_back(searchStamp.value());
    }
    for (auto const& field : fieldsIn) {
      fields.emplace_back(field.value());
    }
  }

  friend bool operator==(DataFrameHeader const&, DataFrameHeader const&);
  friend bool operator!=(DataFrameHeader const&, DataFrameHeader const&);
};

struct DataFrame {
  static FrameTypeIndicator const FRAME_TYPE;

  FrameTypeIndicator frameType{FRAME_TYPE};
  /// Contains metadata about this frame.
  DataFrameHeader header{};
  /// The data for this frame.
  /// The content is compressed according to the specified algorithm
  /// in the `TrackHeaderFrame` for this track (or none).
  DynamicUnwritten data{};

  explicit DataFrame() = default;

  explicit DataFrame(DataFrameHeader const& headerIn, size_t dataSize)
      : header(headerIn), data(dataSize) {}

  friend bool operator==(DataFrame const&, DataFrame const&);
  friend bool operator!=(DataFrame const&, DataFrame const&);
};

struct SearchStampRangeEntry {
  /// The meaning of a and b depends on the `TrackHeaderFrame::searchStampTypes`
  /// entry for this track.
  int64_t a{};
  int64_t b{};

  SearchStampRangeEntry() = default;

  SearchStampRangeEntry(int64_t aIn, int64_t bIn) : a(aIn), b(bIn) {}

  friend bool operator==(SearchStampRangeEntry const& lhs,
                         SearchStampRangeEntry const& rhs) noexcept {
    return lhs.a == rhs.a && lhs.b == rhs.b;
  }

  friend bool operator!=(SearchStampRangeEntry const& lhs,
                         SearchStampRangeEntry const& rhs) noexcept {
    return !(lhs == rhs);
  }
};

using SearchStampRangeList = FixedVector<SearchStampRangeEntry, SEARCH_STAMP_MAX>;

struct SeekFrame {
  static FrameTypeIndicator const FRAME_TYPE;

  FrameTypeIndicator frameType = FRAME_TYPE;
  /// The ID of the track this seek frame describes.
  TrackId trackId{};
  /// The position of the first data frame the entries in this seek frame describe.
  Position firstDataFrameOffset{};
  /// The position of the last data frame the entries in this seek frame describe.
  Position lastDataFrameOffset{};
  /// The position of the seek frame which descibes the data preceding `firstDataFrameOffset`.
  /// The value of `0` indicates there is no seek frame before this.
  Position previousSeekFrameOffset{};
  /// The position of the seek frame which describes data succeeding `lastDataFrameOffset`.
  /// The value of `0` indicates that the seek frame after this is unknown at this time.
  /// This will be `0` unless a post-processing step is applied to fix up these values.
  Position nextSeekFrameOffset{};
  /// The list of entries which describe the search stamp content between
  /// `firstDataFrameOffset` and `lastDataFrameOffset`.
  SearchStampRangeList contained{};

  SeekFrame() = default;

  SeekFrame(TrackId trackIdIn,
            Position firstDataFrameOffsetIn,
            Position lastDataFrameOffsetIn,
            Position previousSeekFrameOffsetIn,
            Position nextSeekFrameOffsetIn,
            Span<SearchStampRangeEntry const> containedIn)
      : trackId(trackIdIn),
        firstDataFrameOffset(firstDataFrameOffsetIn),
        lastDataFrameOffset(lastDataFrameOffsetIn),
        previousSeekFrameOffset(previousSeekFrameOffsetIn),
        nextSeekFrameOffset(nextSeekFrameOffsetIn) {
    for (auto const& entry : containedIn) {
      contained.emplace_back(entry.value());
    }
  }

  friend bool operator==(SeekFrame const& lhs, SeekFrame const& rhs) noexcept;
  friend bool operator!=(SeekFrame const& lhs, SeekFrame const& rhs) noexcept;
};

struct FooterFrame {
  static FrameTypeIndicator const FRAME_TYPE;

  FrameTypeIndicator frameType = FRAME_TYPE;
  /// A list of information about all tracks in the container.
  FooterTrackKnownInfoList tracks;

  FooterFrame() = default;

  explicit FooterFrame(FooterTrackKnownInfoList const& tracksIn) : tracks(tracksIn) {}

  friend bool operator==(FooterFrame const&, FooterFrame const&);
  friend bool operator!=(FooterFrame const&, FooterFrame const&);
};

/// The last piece of a @ref FooterFrame. This is only used when reading.
struct FooterTail {
  static constexpr size_t ENCODED_SIZE = 40U;

  /// This repeated frameSize is intended for those reverse scanning a possibly-complete pod file
  /// to know when the FooterFrame starts. If the FooterFrame is modified in the future and more
  /// fields are added, they must be added before this field.
  uint64_t frameSize;
  /// A SHA-256 digest of all data from the beginning of the frame to the beginning of this field.
  /// This includes bytes between unknown fields -- if a fields are added in a later version,
  /// the checksum will include these new fields, but old versions of the software will still be
  /// capable of validating the footer.
  std::array<uint8_t, 32U> digest;
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_DATA_HPP_
