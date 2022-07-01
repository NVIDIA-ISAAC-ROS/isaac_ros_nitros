/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_WRITER_HPP_
#define NVIDIA_GXF_DW_POD_WRITER_HPP_

#include <memory>
#include <vector>

#include "common/fixed_vector.hpp"
#include "common/optional.hpp"
#include "common/span.hpp"
#include "gxf/dw/pod/bit_flags.hpp"
#include "gxf/dw/pod/data.hpp"
#include "gxf/dw/pod/types.hpp"
#include "gxf/serialization/endpoint.hpp"
#include "gxf/serialization/file.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// The underlying implementation of a @ref Writer. It is not meant to be used directly.
///
/// The majority of the state and buffer management logic lives in this class,
/// while leaf classes are only responsible for moving data to the destination.
class WriterBase {
 public:
  enum class State {
    /// The writer has been opened, but nothing important has been written.
    ///
    ///  - A call to @ref addTrack transitions to @c ADDING_TRACKS
    ///  - A call to @ref finalize transitions to @c FINALIZED (although this file is not useful)
    EMPTY,
    /// Tracks have been added, but no data has been written.
    ///
    ///  - A call to @ref write transitions to @c DATA
    ///  - A call to @ref finalize transitions to @c FINALIZED
    ADDING_TRACKS,
    /// Data is being written.
    ///
    ///  - A call to @ref finalize transitions to @c FINALIZED
    DATA,
    /// @ref finalize has been called -- nothing more can be written.
    FINALIZED,
    /// An unrecoverable error has occurred.
    ERROR,
  };

  /// @see Feature
  struct FeatureType : BitFlags<FeatureType, uint8_t> {
    using BitFlags<FeatureType, uint8_t>::BitFlags;
  };

  /// The features of the underlying write stream.
  struct Feature {
    /// No special features are supported. The stream output can only be appended-to.
    static constexpr FeatureType NONE{0x0};

    /// Is @ref skip implemented? If the underlying stream supports @c SKIP,
    /// then checkpoint frames will skip to known intervals instead of writing
    /// zeros to fill up to that point.
    static constexpr FeatureType SKIP{0x1};

    /// Is @ref overwrite implemented?
    static constexpr FeatureType OVERWRITE{0x2};
  };

  struct K {
    /// The upper limit for the size of frame header information.
    /// This number was chosen based on no real data, but seems to work well enough.
    static constexpr size_t DEFAULT_ENCODE_BUFFER_SIZE = 2UL * 1024UL * 1024UL;

    /// How many tracks should be reserved?
    static constexpr size_t DEFAULT_TRACK_RESERVATIONS = 32U;

    /// The default number of frames between seek frames.
    static constexpr size_t DEFAULT_SEEK_INTERVAL_FRAME_COUNT = 1000U;

    /// The default distance between seek frames.
    static constexpr size_t DEFAULT_SEEK_INTERVAL_FILE_DISTANCE = 100UL * 1024UL * 1024UL;
  };

  WriterBase(WriterBase const&) = delete;
  WriterBase& operator=(WriterBase const&) = delete;

  virtual ~WriterBase() noexcept;

  /// @see Writer::sync
  virtual void sync() = 0;

  /// @see Writer::addTrack
  TrackId addTrack(Span<EncodedFieldView const> fields,
                   Span<SearchStampType const> searchStampTypes,
                   Span<uint8_t const> unstructuredData);

  /// @see Writer::write
  void write(TrackId trackId,
             BundleId bundleId,
             Span<uint8_t const> data,
             Span<SearchStamp const> searchStamps,
             Span<EncodedFieldView const> fields);

  /// @see Writer::finalize
  void finalize();

  /// @see Writer::getCursorPosition
  Position getCursorPosition() const noexcept;

 protected:
  using SegmentSpan = Span<Span<uint8_t const> const>;

  /// @param features The features of the subclass.
  explicit WriterBase(FeatureType features, std::optional<FileVersion> fileVersion);

  /// Append data in the given @a segments to the file.
  virtual void doAppendData(SegmentSpan segments) = 0;

  /// Skip forward by a @a distance.
  ///
  /// @throw std::runtime_error if the underlying stream does not support skipping.
  virtual void doSkip(size_t distance) = 0;

  /// Overwrite the data at @a position with the given @a segments.
  ///
  /// @throw std::runtime_error if the underlying stream does not support overwriting.
  virtual void doOverwriteData(Position position, SegmentSpan segments) = 0;

 private:
  /// Used in @c TrackControl to track per @c SearchStamp information.
  class SearchStampIntermediateInfo {
   public:
    explicit SearchStampIntermediateInfo(SearchStampType const type) noexcept;

    /// Update the information with the given @a value.
    ///
    /// @param trackId ID used for logging purposes
    /// @param writtenPosition Position of the data frame that was just written
    /// @param firstFrame Was this the first frame written for this segment?
    /// @param value The value of the search stamps written
    /// @param stampIndex The index of the stamp in the track's stamp list
    void update(TrackId const trackId,
                Position const writtenPosition,
                bool const firstFrame,
                SearchStamp const& value,
                size_t const stampIndex) noexcept;

    SearchStampType type_;

    SearchStampRangeEntry values_;

    /// Have the range entries been monotonic up to this point?
    bool monotonic_;
  };

  class TrackControl {
   public:
    template <typename TSearchStampTypeRange>
    explicit TrackControl(TrackId, Position, TSearchStampTypeRange const&);

    /// Called after a data frame is written. This function is responsible for updating
    /// the seeking information.
    void noteDataWrite(Position dataPosition, Span<SearchStamp const> searchStamps);

    /// Called after a seek frame is written.
    void noteSeekWrite(Position seekPosition) noexcept;

    /// Returns @c true if this track should have a seek frame written for it.
    ///
    /// @param stronglySuggest If @c true, this will return @c true if a single data frame
    ///                        has been written since the last seek frame. See the parameter
    ///                        of the same name in @c WriterBase::maybeWriteSeekFrame for
    ///                        more information.
    bool wantsSeekFrameWritten(bool stronglySuggest) const noexcept;

    /// @see K::DEFAULT_SEEK_INTERVAL_FRAME_COUNT
    size_t getSeekIntervalFrameCount() const noexcept;

    /// @see K::DEFAULT_SEEK_INTERVAL_FILE_DISTANCE
    size_t getSeekIntervalFileDistance() const noexcept;

    TrackId trackId_;
    Position headerLocation_;

    /// The number of data frames that have been written so far.
    size_t dataFrameCount_;

    /// The number of data frames that were written last seek frame
    size_t dataFrameCountLastSeekFrame_;

    FixedVector<SearchStampIntermediateInfo, SEARCH_STAMP_MAX> searchStampInfo_;

    Position firstDataFrameLocation_;
    Position lastDataFrameLocation_;
    Position previousSeekFrameLocation_;
    Position lastSeekFrameLocation_;
  };

  TrackControl& getTrack(TrackId trackId);

  Position appendData(SegmentSpan segments);
  Position appendData(Span<uint8_t const> segment);
  Position appendData(Span<uint8_t const> segment1, Span<uint8_t const> segment2);

  void skip(size_t distance);

  void maybeWriteOpening(State newState);

  void maybeRewritePodHeader();

  /// @param trackCtl The track to write the seek frame for.
  /// @param stronglySuggest If @c true, it is strongly suggested that we write the seek frame
  ///                        at this moment (usually because we are finalizing the file).
  ///                        This still might not write a seek frame if seeking
  ///                        is disabled or if no data has been written for the track.
  void maybeWriteSeekFrame(TrackControl& trackCtl, bool stronglySuggest = false);

  FileVersion fileVersion_;
  FeatureType features_;
  State state_;
  Position position_;
  std::vector<TrackControl> tracks_;
  std::vector<uint8_t> buffer_;
};

/// Writes to a local @ref File instance.
class WriterLocalFile final : public WriterBase {
 public:
  explicit WriterLocalFile(Handle<File> file, std::optional<FileVersion> fileVersion);

  ~WriterLocalFile() noexcept override;

  /// @see WriterBase::sync
  void sync() override;

 protected:
  /// @see WriterBase::doAppendData
  void doAppendData(SegmentSpan segments) override;

  /// @see WriterBase::doSkip
  void doSkip(size_t distance) override;

  /// @see WriterBase::doOverwriteData
  void doOverwriteData(Position position, SegmentSpan segments) override;

 private:
  Handle<File> file_;
};

class Writer final {
 public:
  using HandleType = WriterBase*;

  Writer() noexcept;

  Writer(Writer&&) noexcept;

  Writer& operator=(Writer&&) noexcept;

  ~Writer() noexcept;

  /// Create an instance writing to the given output @a stream.
  static Writer make(Handle<Endpoint> stream, std::optional<FileVersion> fileVersion = {});

  /// Create an instance writing to the provided local @a file.
  static Writer makeLocal(Handle<File> file, std::optional<FileVersion> fileVersion = {});

  /// @{
  /// Adopt responsibility for an existing @a handle.
  ///
  /// @param handle A handle to an instance.
  ///               This value should have come from a previous call to @ref release.
  static Writer adopt(HandleType handle) noexcept;

  /// Release ownership of the underlying objects. It is now the caller's responsibility.
  ///
  /// @see adopt
  HandleType release() noexcept { return impl_.release(); }
  /// @}

  /// Reset this writer to the empty state.
  void reset() noexcept { impl_.reset(); }

  /// Add a track.
  ///
  /// @param fields The user-defined fields to add to the track header.
  /// @param searchStampTypes The types of search stamps for this track.
  ///                         This determines how seek entries are built for this track
  ///                         (see @ref SearchStampType for more information).
  ///                         The same count of types passed in this parameter need to be
  ///                         provided to future calls to @ref write.
  /// @param unstructuredData Additional unstructured header information.
  ///                         This is meant to be used for formats which have a serialized
  ///                         form of header. For example, H.264 has its own header format
  ///                         that should be written in this field.
  ///
  /// @return the index of the created track.
  ///
  /// @throw std::invalid_argument if @a fields or @a searchStampTypes is too large.
  /// @throw std::runtime_error if the writer has already been finalized.
  /// @throw std::runtime_error if you have reached the maximum number of tracks.
  ///
  /// @note
  /// It is legal to call this function after track data has been written,
  /// but doing so is discouraged. If the stream backing this writer is seekable,
  /// the header will be fixed up with a call to @ref sync to avoid potential data
  /// corruption, which is more expensive. If the stream is not seekable, the pod
  /// header will not contain the added track, but future checkpoints and the footer will.
  Expected<TrackId> addTrack(Span<EncodedFieldView const> fields,
                             Span<SearchStampType const> searchStampTypes,
                             Span<uint8_t const> unstructuredData = Span<uint8_t const>());

  /// Write @a data for the given @a trackId.
  ///
  /// @param trackId The ID of the track to write to (previously returned from @ref addTrack call).
  /// @param bundleId The "frame bundle" of this piece of data, generally used to group frames
  ///                 together for the purpose of presentation.
  /// @param data The content to write. The minimum size of data is 0, which can be useful for
  ///             writing marker frames with only metadata.
  /// @param searchStamps The values of search stamps used to build seek information.
  ///                     The values in this list are interpreted per the @c searchStampTypes
  ///                     passed to the @ref addTrack call which created this @a trackId.
  ///                     The list size must be the same.
  /// @param fields The user-defined fields to add to this data frame's metadata.
  ///
  /// @throw std::invalid_argument if @a searchStamps does not have the same count of values
  ///                                 as @c searchStampTypes from the call to @ref addTrack.
  /// @throw std::runtime_error if the writer has already been finalized.
  Expected<void> write(TrackId trackId,
                       BundleId bundleId,
                       Span<uint8_t const> data,
                       Span<SearchStamp const> searchStamps,
                       Span<EncodedFieldView const> fields);

  /// Write footer information, ending the write.
  /// After this call returns, write operations such as @ref write and @ref addTrack
  /// are no longer valid.
  ///
  /// @note
  /// This is @e not automatically called for you by the destructor.
  /// You must call it manually when you want the footer to be written.
  ///
  /// @throw std::runtime_error if the writer has already been finalized.
  Expected<void> finalize();

  /// Sync the contents to backing storage.
  /// This call blocks until until the backing device reports that all data transfer
  /// has been completed. Calls to this function are still legal after @ref finalize.
  ///
  /// @note
  /// This function is @e not called automatically by the destructor because it is a
  /// potentially-blocking operation. If file integrity in power loss is important to you,
  /// this must be called manually.
  Expected<void> sync();

  /// Get the current cursor position for the underlying stream
  /// The value returned from this function should be used as advisory-only in diagnostics.
  /// The next frame might not be written at the current end of file due to metadata-only
  /// frames like seek frames and checkpoints.
  Position getCursorPosition() const noexcept;

 private:
  explicit Writer(std::unique_ptr<WriterBase> impl) noexcept;

  /// Check that this instance is valid for use.
  ///
  /// @throw std::runtime_error if this instance was default-initialized or has been moved-from.
  void ensureValid() const;

  std::unique_ptr<WriterBase> impl_;
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_POD_WRITER_HPP_
