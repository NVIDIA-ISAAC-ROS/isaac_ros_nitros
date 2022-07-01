/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_READER_HPP_
#define NVIDIA_GXF_DW_POD_READER_HPP_

#include <memory>
#include <utility>

#include "common/fixed_vector.hpp"
#include "common/optional.hpp"
#include "common/span.hpp"
#include "gxf/dw/pod/bit_flags.hpp"
#include "gxf/dw/pod/data.hpp"
#include "gxf/dw/pod/header.hpp"
#include "gxf/dw/pod/types.hpp"
#include "gxf/serialization/endpoint.hpp"
#include "gxf/serialization/file.hpp"

namespace nvidia {
namespace gxf {
namespace pod {

/// The status returned from traversal and seeking operations.
enum class ReaderStatus : uint8_t {
  /// Success -- the data is ready to read.
  READY,
  /// The call was interrupted, but did not necessarily fail.
  /// A retry of the same operation might succeed.
  INTERRUPTED,
  /// The call reached the end of the content.
  CONTENT_END,
};

/// Base class for @ref pod::Reader implementations.
/// Most documentation for entry-point functions lives in that class.
///
/// The base class provides the state machine for the reader -- @c next, @c currentHeader,
/// @c readData, and internal error management. Leaf classes are responsible for management
/// of the read buffer.
class ReaderBase {
 public:
  enum class State {
    /// The reader has been opened, but no content has been read.
    OPENED,
    /// The stream is positioned at the beginning of a frame.
    /// The next call should be to @c currentHeader.
    FRAME_START,
    /// The header has been read and we are ready to @c readData or @c next to skip.
    FRAME_DATA,
    /// All data from the current frame has been consumed.
    FRAME_END,
    /// The reader has reached the end of content.
    END,
    /// An unrecoverable error has been encountered.
    /// All operations will throw @c InvalidStateException.
    UNRECOVERABLE_ERROR,
  };

  /// @see Feature
  struct FeatureType : BitFlags<FeatureType, uint8_t> {
    using BitFlags<FeatureType, uint8_t>::BitFlags;
  };

  /// Indicators to determine what operations are supported by the
  /// @ref pod::ReaderBase implementation.
  struct Feature {
    /// No special features are supported. The source can only be read-from linearly.
    static constexpr FeatureType NONE{0x0};

    /// Does the underlying source support seek operations?
    static constexpr FeatureType SEEKABLE{0x1};
  };

  ReaderBase(ReaderBase const&) = delete;
  ReaderBase& operator=(ReaderBase const&) = delete;

  virtual ~ReaderBase() noexcept;

  /// @see Reader::next
  ReaderStatus next();

  /// Get the encoded span of the current header.
  /// This function has limited applicability outside of unit testing.
  ///
  /// @returns The indicator for the current frame type and a span containing the encoded
  ///          bytes of the current header if we are currently pointing the beginning of a
  ///          frame (the state is @c State::FRAME_START). If this instance is not pointing to
  ///          the beginning of a frame, this will return @c FRAME_TYPE_NONE and an empty span.
  ///
  /// @see currentHeader
  std::pair<FrameTypeIndicator, Span<uint8_t const>> currentHeaderRaw() const;

  /// @see Reader::currentHeader
  HeaderView currentHeader();

  /// @see Reader::readData
  Span<uint8_t const> readData(Span<uint8_t> buffer);

  /// @see Reader::canSeek
  bool canSeek() const noexcept { return features_.isSet(Feature::SEEKABLE); }

  /// @see Reader::seekTo
  ReaderStatus seekTo(TrackId trackId, size_t searchStampIndex, SearchStamp goalValue);

 protected:
  struct ReadResult {
    ReaderStatus status;
    Span<uint8_t const> data;

    constexpr explicit ReadResult(ReaderStatus statusIn, Span<uint8_t const> dataIn) noexcept
        : status(statusIn), data(std::move(dataIn)) {}

    constexpr bool good() const noexcept { return status == ReaderStatus::READY; }
  };

  explicit ReaderBase(FeatureType features) noexcept;

  /// Get the current position of the cursor in the source.
  /// This must have a value, even if the underlying stream does not track its own position.
  virtual Position getCursorPosition() const noexcept = 0;

  /// Set the position of the cursor in the source.
  /// If this is not overridden in a derived class, it will always throw an @ref std::runtime_error.
  /// This will only be called if the @c Feature::SEEKABLE flag is set.
  virtual void setCursorPosition(Position newLocation);

  /// Move the cursor forward by @a byteCount bytes.
  ///
  /// @param byteCount The maximum number of bytes to advance the stream by.
  ///
  /// @return the number of bytes the underlying stream advanced by.
  ///         In the success case, this value should be equal to @a byteCount.
  ///         It is not an error to move forward by fewer bytes than requested, even @c 0 bytes.
  ///         An exception should only be thrown if the cursor can not be moved again.
  virtual size_t moveCursorForward(size_t byteCount) = 0;

  /// Load at least @a minBytes from the current source position.
  /// Note that this does not affect the position in the stream.
  /// Repeated attempts to @c loadCurrentBuffer with the same @a minBytes
  /// should return the same region.
  ///
  /// @param minBytes The minimum number of bytes to load.
  ///                 If fewer than @a minBytes can be loaded,
  ///                 the result should have @c Status::INTERRUPTED.
  ///
  /// @return A result with @c Status::READY if at least @a minBytes have been read.
  virtual ReadResult loadCurrentBuffer(size_t minBytes) = 0;

  /// Get the current buffer without performing any load operations.
  virtual Span<uint8_t const> getCurrentBuffer() const noexcept = 0;

  /// Attempt to read @a maxBytes from the source, potentially using the user-provided
  /// @a destination for storage. See @ref pod::Reader::readData for more information on
  // buffer usage.
  virtual Span<uint8_t const> readAdvanceBuffer(size_t maxBytes, Span<uint8_t> destination) = 0;

 private:
  class TrackInfo final {
   public:
    explicit TrackInfo(TrackKnownInfo const& knownInfo);

    explicit TrackInfo(TrackId trackId, Position headerLocation);

    ~TrackInfo() noexcept;

    TrackId trackId_;
    Position headerLocation_;
    Position lastDataFrame_;
    Position lastSeekFrame_;
    FixedVector<SearchStampType, SEARCH_STAMP_MAX> searchStampTypes_;
    DataCompressionMode compressionMode_;
  };

  /// The @c seekTo parameters captured as a structure.
  struct SeekGoal final {
    TrackId trackId;
    size_t searchStampIndex;
    SearchStamp goalValue;

    explicit constexpr SeekGoal(TrackId trackIdIn,
                                size_t searchStampIndexIn,
                                SearchStamp goalValueIn)
        : trackId(trackIdIn), searchStampIndex(searchStampIndexIn), goalValue(goalValueIn) {}

    friend constexpr bool operator==(SeekGoal const& lhs, SeekGoal const& rhs) noexcept {
      return lhs.trackId == rhs.trackId
          && lhs.searchStampIndex == rhs.searchStampIndex
          && lhs.goalValue == rhs.goalValue;
    }

    friend constexpr bool operator!=(SeekGoal const& lhs, SeekGoal const& rhs) noexcept {
      return !(lhs == rhs);
    }
  };

  enum class SeekState : uint8_t {
    /// @c seekSkipBack
    SKIPPING_BACK,
    /// @c seekScanForward
    SCANNING_FORWARD,
    /// Done: The proper frame has been found
    FOUND,
    /// Done: The proper frame was not found -- it is not in this file
    NOT_FOUND,
  };

  /// Keeps a cache of information about current seek operation. Seeking can end up making
  /// a significant number of small reads, so the chance of being interrupted is quite high.
  struct SeekOpCache final {
    /// Current goal of the seek.
    SeekGoal goal;

    SearchStampType stampType;

    SeekState state;

    /// When @c state is @c SKIPPING_BACK, this refers to the next seek frame
    /// we want to traverse to. This is not used in other states.
    Position nextSeekFrame;

    /// The last position that could be relevant to the search.
    Position lastRelevantPosition;

    /// The earliest location that a flags query matched.
    Position earliestFlagsMatched;

    explicit SeekOpCache(SeekGoal const& goalIn,
                         SearchStampType stampTypeIn,
                         Position lastSeekFrame) noexcept
        : goal(goalIn),
          stampType(stampTypeIn),
          state(SeekState::SKIPPING_BACK),
          nextSeekFrame(lastSeekFrame),
          lastRelevantPosition(lastSeekFrame),
          earliestFlagsMatched(POSITION_NONE) {}

    ~SeekOpCache() noexcept;
  };

  ReadResult loadNextFrameHeader();

  ReaderStatus loadPodHeaders();

  ReaderStatus endCurrentFrame();

  /// Clear current frame states and set the cursor to @a newPosition.
  /// This will throw @ref std::runtime_error if the underlying stream is not seekable.
  ///
  /// @param newPosition The new position to set the cursor to.
  ///                    This is assumed to be the start of a frame of some kind.
  void repositionTo(Position newPosition);

  TrackInfo const& getTrackInfo(TrackId trackId) const;
  TrackInfo& getTrackInfo(TrackId trackId);

  /// Update the track info after encountering the header.
  TrackInfo const& updateTrackInfo(Position headerLocation, TrackHeaderFrame const& frame);

  /// Update the track info after encountering a seek frame.
  void updateTrackInfo(Position frameLocation, SeekFrame const& frame);

  /// Seek Initialization: potentially initializes the seek cache or re-uses the
  /// existing one if the parameters match what was set before.
  SeekOpCache& seekInitCache(TrackId trackId, size_t searchStampIndex, SearchStamp goalValue);

  /// Seek Step 1:
  /// The reader is skipping backwards in large chunks across the linked list of seek frames
  ReaderStatus seekSkipBack(SeekOpCache& seekCache);

  /// Seek Step 2:
  /// The reader is scanning forwards frame-by-frame to find the exact position to start at
  ReaderStatus seekScanForward(SeekOpCache& seekCache);

  FeatureType features_;

  State state_;
  size_t currentFrameSize_;        //!< The total size of the current frame
  size_t currentFrameCursor_;      //!< The read offset of the current frame
  size_t currentFrameDataOffset_;  //!< The start of the data segment of the current frame

  FileVersion fileVersion_;
  std::optional<uint64_t> checkpointInterval_;

  FixedVector<TrackInfo, TRACK_COUNT_MAX> tracks_;
  TrackHeaderFrame cachedTrackHeaderFrame_;
  DataFrame cachedDataFrame_;

  std::optional<SeekOpCache> seek_;
};

/// Get a string representation of the given @a state.
char const* getName(ReaderBase::State state);

/// Reader from a local @ref File instance.
class ReaderLocalFile final : public ReaderBase {
 public:
  struct K {
    /// The size of the read buffer.
    /// 2MB is the default buffer size as it provides the best performance for NVMEe SSDs.
    static constexpr size_t DEFAULT_BUFFER_SIZE = 2UL * 1024UL * 1024UL;
  };

  explicit ReaderLocalFile(Handle<File> file);

  ~ReaderLocalFile() noexcept override;

 protected:
  /// @see ReaderBase::getCursorPosition
  Position getCursorPosition() const noexcept override;

  /// @see ReaderBase::setCursorPosition
  void setCursorPosition(Position newLocation) noexcept override;

  /// @see ReaderBase::moveCursorForward
  size_t moveCursorForward(size_t byteCount) override;

  /// @see ReaderBase::loadCurrentBuffer
  ReadResult loadCurrentBuffer(size_t minBytes) override;

  /// @see ReaderBase::getCurrentBuffer
  Span<uint8_t const> getCurrentBuffer() const noexcept override;

  /// @see ReaderBase::readAdvanceBuffer
  Span<uint8_t const> readAdvanceBuffer(size_t maxBytes, Span<uint8_t> destination) override;

 private:
  Handle<File> file_;
  uint8_t buffer_[K::DEFAULT_BUFFER_SIZE];
};

class Reader final {
 public:
  using HandleType = ReaderBase*;

  Reader() noexcept;

  Reader(Reader&&) noexcept;

  Reader& operator=(Reader&&) noexcept;

  ~Reader() noexcept;

  /// @{
  /// Create a reader from the @a source stream.
  ///
  /// @param source The stream to read from. The only strict requirement on the stream is that
  ///               it is reliable -- it will never lose data in transport without signalling.
  ///               If the stream is not seekable, then seeking operations are unsupported.
  /// @param initialOffset If specified, the offset of the @a source. This can either be @c 0 or
  ///                      positioned immediately after the @c FileHeader. The latter case is
  ///                      useful when demultiplexing on a stream which is not seekable -- after
  ///                      reading the magic number from the @c FileHeader to identify a DW Pod
  ///                      formatted file, pass the offset in to indicate that it has been read.
  static Reader make(Handle<Endpoint> source, Position initialOffset = Position(0U));
  /// @}

  /// Create a reader from a local readable @a file.
  ///
  /// @warning
  /// The implementation assumes the storage backing @a file is always accessible.
  /// For non-local files, such as those connected via a network mount, the application
  /// will crash with @c SIGBUS if that mount is disconnected.
  static Reader makeLocal(Handle<File> file);

  /// @{
  /// Adopt responsibility for an existing @a handle.
  ///
  /// @param handle A handle to an instance. This value should have come from a previous call
  //                to @ref release.
  static Reader adopt(HandleType handle) noexcept;

  /// Release ownership of the underlying object. It is now the caller's responsibility.
  ///
  /// @see adopt
  HandleType release() noexcept { return impl_.release(); }
  /// @}

  /// Reset this reader to the empty state.
  void reset() noexcept { impl_.reset(); }

  /// Attempt to traverse to the next frame.
  ///
  /// @return
  /// * @c READY -- The next frame was found. A call to @ref getDataHeader will get the current
  ///               header and a call to @ref readData will return data.
  /// * @c INTERRUPTED -- The call was interrupted by something. Usually this means the underlying
  ///                     write operation was interrupted by a signal and got @c EINTR.
  /// * @c CONTENT_END -- We have reached the end of the content. Future calls to this function
  ///                     will continue to return @c CONTENT_END.
  Expected<ReaderStatus> next();

  /// Get the current header.
  ///
  /// The contents of the returned value are invalidated when @ref next is called.
  ///
  /// @throw std::runtime_error if the current state of the reader is not @c READY.
  Expected<HeaderView> currentHeader() const;

  /// Read the contents of the current frame, potentially using the provided @a buffer.
  /// This function shall only be called after a call to @ref currentHeader.
  ///
  /// To look at the data, you should @e always use the buffer returned from from this function
  /// and @e never use the @a buffer you provided as an argument. If the data fully exists in the
  /// implementation's in-memory buffer, then a view of that buffer will be returned instead of
  /// copying data to the user-provided one.
  ///
  /// @param buffer The buffer to potentially copy read data in to. If this buffer has size 0,
  ///               data can still be returned from the internal buffer, but only data which is
  ///               resident in memory will be returned.
  ///
  /// @throw std::runtime_error if the reader is not currently pointing at a frame which holds
  ///        data. Note that this is not thrown when the data length is 0 (in this case, an empty
  ///        span is returned), but when this instance is pointing at something that could not
  ///        possibly hold data, such as if @ref next returned @c CONTENT_END.
  /// @throw std::runtime_error if the provided @a buffer must be used for data reading and is
  ///                           too small to hold the data you wish to read.
  Expected<Span<uint8_t const>> readData(Span<uint8_t> buffer) const;

  /// Check if seek operations are available for this reader.
  bool canSeek() const;

  /// Attempt to move this reader to the data frame which matches the search criteria.
  /// If @a searchStampIndex refers to a @c SearchStampType::kRange, then a successful seek
  /// operation will point to the first data frame where the corresponding search stamp is not
  /// less than @a goalValue (similar to @c std::lower_bound). If @a searchStampIndex refers to
  /// a @c SearchStampType::kFlags, then a successful seek operation will point to the first data
  /// frame where the search stamp matches @e any of the bits of @a goalValue.
  ///
  /// Seek operations assume monotonically increasing values for @c SearchStampType::kRange entries
  /// in the source file. If this is not true, the exact behavior of this function can not be
  /// relied upon. While it will end in a position which matches the search criteria, it is not
  /// guaranteed to be the first position which matches the criteria. Exactly which position in the
  /// file the reader will find is unspecified.
  ///
  /// @param trackId The track to search the stamps for.
  /// @param searchStampIndex The index of the search stamp for the given track to search for.
  ///                         The @c SearchStampType is based on this, which sets the meaning of
  ///                         @a goalValue.
  /// @param goalValue The value to search for.
  ///
  /// @return
  /// * @c READY -- the @a goalValue was successfully seeked to. The next step is to call
  ///               @ref currentHeader to load the header for the data frame.
  /// * @c INTERRUPTED -- The seek operation was interrupted by something (see @ref next for
  ///                     more information). Call this function again with the same parameters
  ///                     to resume seeking. If you do not wish to resume seeking, the underlying
  ///                     reader is left in a well-defined state in an unspecified location.
  ///                     Call @ref next to resume playback.
  /// * @c CONTENT_END -- The seek operation successfully determined that no content matches the
  ///                     search criteria. The reader is left at the end of the file and behaves
  ///                     as if @ref next had reached the same point.
  ///
  /// @throw std::runtime_error if seeking is not supported (result of @ref canSeek is false).
  /// @throw std::invalid_argument if @a trackId does not exist or if @a searchStampIndex does
  ///                              not exist for that track.
  /// @throw std::invalid_argument if @a searchStampIndex refers to a @c SearchStampType::kFlags
  ///                              and @a goalValue is @c 0 -- this will never successfully match.
  Expected<ReaderStatus> seekTo(TrackId trackId, size_t searchStampIndex, SearchStamp goalValue);

 private:
  explicit Reader(std::unique_ptr<ReaderBase> ptr);

  void ensureValid() const;

  std::unique_ptr<ReaderBase> impl_;
};

}  // namespace pod
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_READER_HPP_
