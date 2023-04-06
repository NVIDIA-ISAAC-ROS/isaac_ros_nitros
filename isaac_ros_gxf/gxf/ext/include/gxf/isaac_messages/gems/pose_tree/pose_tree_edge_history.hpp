/*
Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <shared_mutex>

#include "engine/core/math/pose3.hpp"
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace isaac {

// Class that stores an history of Poses (using pre-allocated memory and a cyclic buffer) and
// provides helper function to add a new pose or query the pose at a given time.
class PoseTreeEdgeHistory {
 public:
  // Interpolation method for accessing poses in the pose tree
  enum class AccessMethod {
    // Gets the value of the closest sample
    kNearest,
    // Interpolates linearly between adjacent samples. If the query is outside the validity range,
    // the closest pose will be returned.
    kInterpolateLinearly,
    // Inter- or extrapolates linearly based on neighbouring samples. This require at least two
    // valid poses or Error::kOutOfRange will be returned.
    kExtrapolateLinearly,
    // Interpolates with slerp between adjacent samples. If the query is outside the validity range,
    // the closest pose will be returned.
    kInterpolateSlerp,
    // Inter- or extrapolates with slerp based on neighbouring samples. This require at least two
    // valid poses or Error::kOutOfRange will be returned.
    kExtrapolateSlerp,
    // Use the latest Pose before a given time.
    kPrevious,
    // Fallback to the default interpolation
    kDefault,
  };

  // Error codes used by this class.
  enum class Error {
    // kInvalidArgument is returned when a function is called with argument that does not make sense
    // such as querying a pose outside the valid range or provide a wrong interpolation method.
    kInvalidArgument,
    // kOutOfOrder is returned when a set/disconnected called is made with a version or time
    // lower than the latest TimedPose. Both time and version must be stricly increasing.
    kOutOfOrder,
    // kFramesNotLinked is returns if a get query is made and the tree is not connected at the given
    // time and version of the edge.
    kFramesNotLinked,
    // kOutOfRange is returned if not enough poses are available to do extrapolation or if the
    // available buffer is too small to store a new pose.
    kOutOfRange,
  };

  // Expected type used by this class.
  template <typename T>
  using Expected = nvidia::Expected<T, Error>;
  // Unexpected type used by this class.
  using Unexpected = nvidia::Unexpected<Error>;

  // Type used to uniquely identify a frame.
  using frame_t = uint64_t;
  // Type used for versioning the edge.
  using version_t = uint64_t;

  // Helper structure to store the pose at a given time on the edge.
  struct TimedPose {
    // 3D pose that transforms the lhs frame into the rhs frame.
    ::isaac::Pose3d pose;
    // Time of the pose. Needs to be strictly increasing.
    double time;
    // Version ID of the pose. Needs to be strictly increasing.
    version_t version;
    // If false, then it marks the edge as being disconnected from this current time. The pose does
    // not matter.
    bool valid;
  };

  // Default constructor used to be able to pre-allocate memory.
  PoseTreeEdgeHistory() = default;

  // Constructor to actually uses the object:
  // `buffer` is a pre allocated buffer that can old `maximum_size` elements.
  PoseTreeEdgeHistory(frame_t lhs, frame_t rhs, int32_t maximum_size, TimedPose* buffer);
  PoseTreeEdgeHistory(frame_t lhs, frame_t rhs, int32_t maximum_size, AccessMethod access_method,
                      TimedPose* buffer);

  // Sets the pose at a given amount of time. If the array is empty Error::kOutOfMemory will be
  // returned, otherwise if a pose already exist and `time` or `version` <= pose.time/version then
  // Error::kOutOfOrder is returned. Otherwise it will succeed, and if the history already
  // contained maximum_size_ element, then the oldest pose will be forgotten.
  Expected<void> set(double time, const ::isaac::Pose3d& pose, version_t version);

  // Returns the TimedPose at a given position. If index is negative Error::kInvalidArgument will be
  // returned, and if index >= size, then Error::kOutOfRange will be returned.
  Expected<TimedPose> at(int32_t index) const;

  // Returns the Pose3d at a given time using the given version of the PoseTree.
  // If no pose existed at the given time, Error::kFramesNotLinked will be returned.
  // The desired method can be provided, for kExtrapolateLinearly, at least two poses are required.
  Expected<::isaac::Pose3d> get(double time, AccessMethod method, version_t version) const;

  // Disconnects a frame at a given time.
  Expected<void> disconnect(double time, version_t version);

  // Returns whether or not the frame are current connected
  bool connected() const;

  // Resets the history, all the poses will be erased.
  void reset();

  // Returns the information about the latest pose
  Expected<TimedPose> latest() const;

  // Returns a pointer to the buffer.
  const TimedPose* data() const {
    return edges_info_;
  }

  // Returns the current size.
  int32_t size() const {
    return size_;
  }

  // Returns the maximum number of poses this edge can contain.
  int32_t maximum_size() const {
    return maximum_size_;
  }

  // This edge reprensent the transformation from the rhs frame to the lhs frame. This function
  // returns the uid of the lhs frame.
  frame_t lhs() const {
    return lhs_;
  }

  // This edge reprensent the transformation from the rhs frame to the lhs frame. This function
  // returns the uid of the rhs frame.
  frame_t rhs() const {
    return rhs_;
  }

 private:
  // Reserves a new pose for a given time/version and returns it's index.
  // If there exists another pose with a later time/version, it returns kOutOfOrder.
  Expected<int32_t> reserveNewPose(double time, version_t version);

  // Mutex to protect changes to this object.
  mutable std::shared_timed_mutex mutex_;
  // Pointers to the buffer that contains the list of TimedPose (we use a circulat buffer to access
  // it).
  TimedPose* edges_info_ = nullptr;
  // Name of the frame this edge connects to.
  frame_t lhs_, rhs_;
  // Size of the buffer, aka maximum number of elements this edge can store in the same time.
  int32_t maximum_size_ = 0;
  // Current number of poses on this edge.
  int32_t size_ = 0;
  // Position of the first pose in the circular buffer,
  int32_t pos_ = 0;
  // Default access method.
  AccessMethod default_access_method_;
};

}  // namespace isaac
}  // namespace nvidia
