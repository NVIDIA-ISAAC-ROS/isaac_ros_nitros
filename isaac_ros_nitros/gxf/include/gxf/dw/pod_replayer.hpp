/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_REPLAYER_HPP_
#define NVIDIA_GXF_DW_POD_REPLAYER_HPP_

#include <unordered_map>

#include "common/fixed_vector.hpp"
#include "gxf/dw/pod/reader.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/serialization/file.hpp"
#include "gxf/serialization/serialization_buffer.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

// Replays messages from a DriveWorks Pod
class PodReplayer : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  // Structure for organizing frame metadata
  struct FrameInfo {
    pod::ReaderStatus status;
    pod::HeaderView header;
  };

  // Decodes message channel metadata from frame
  Expected<void> processTrackHeaderFrame();
  // Decodes entity from frame
  Expected<void> processDataFrame();
  // Skips to the next frame
  Expected<void> processUnknownFrame();

  // Reads from Pod until the next data frame
  // Populates `frame_` with output from `pod::Reader::next()` and `pod::Reader::currentHeader()`
  Expected<void> nextFrame();

  Parameter<FixedVector<Handle<Transmitter>, kMaxComponents>> transmitters_;
  Parameter<Handle<EntitySerializer>> entity_serializer_;
  Parameter<Handle<SerializationBuffer>> serialization_buffer_;
  Parameter<Handle<File>> file_;

  // Table that maps track ID to a transmitter
  std::unordered_map<uint64_t, Handle<Transmitter>> channel_map_;
  // Current frame information
  FrameInfo frame_;
  // Pod reader
  pod::Reader reader_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_REPLAYER_HPP_
