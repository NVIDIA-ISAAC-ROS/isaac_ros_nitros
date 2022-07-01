/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_INFO_HPP_
#define NVIDIA_GXF_DW_POD_INFO_HPP_

#include "common/fixed_vector.hpp"
#include "gxf/dw/pod/field.hpp"

namespace nvidia {
namespace gxf {

// Message component used to communicate with PodRecorder
struct PodInfo {
  static constexpr size_t kMaxDescriptionLength = 1024;
  static constexpr size_t kMaxTags = 64;

  enum class State {
    kUnknown = 0,
    kStop = 1,
    kRecord = 2,
    kError = 3,
  };

  // Recording state
  // Can be used to manually start/stop recording
  State state{State::kUnknown};
  // Recording UUID
  pod::NumericField<uint64_t> uuid1{"uuid1", 0};
  pod::NumericField<uint64_t> uuid2{"uuid2", 0};
  // Recording timestamp in nanoseconds
  pod::NumericField<uint64_t> timestamp{"timestamp", 0};
  // Recording duration in nanoseconds
  pod::NumericField<uint64_t> duration{"duration", 0};
  // Vehicle identification number
  pod::StringField<> vin{"vin", ""};
  // Recording author
  pod::StringField<> author{"author", ""};
  // Recording title
  pod::StringField<> title{"title", ""};
  // Recording description
  pod::StringField<FixedString<kMaxDescriptionLength>> description{"description", ""};
  // Recording tags
  FixedVector<pod::StringField<>, kMaxTags> tags{};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_INFO_HPP_
