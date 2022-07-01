/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_DW_POD_RECORDER_HPP_
#define NVIDIA_GXF_DW_POD_RECORDER_HPP_

#include <unordered_map>

#include "common/fixed_vector.hpp"
#include "gxf/dw/pod/writer.hpp"
#include "gxf/dw/pod_info.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/serialization/file.hpp"
#include "gxf/serialization/serialization_buffer.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

// Records messages to a DriveWorks Pod
class PodRecorder : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  // Structure for organizing message channel metadata
  struct ChannelInfo {
    pod::TrackId track_id;
    size_t message_count;
  };

  // Opens the file and starts recording
  Expected<void> startRecording();
  // Stops recording and closes the file
  Expected<void> stopRecording();
  // Receives and discards all available messages from all receivers
  Expected<void> clearAllReceivers();
  // Receives and records all available messages from all receivers
  Expected<void> recordAllReceivers(pod::BundleId bundle_id);
  // Receives and records all available messages from a receiver
  Expected<void> recordReceiver(Handle<Receiver> receiver, pod::BundleId bundle_id);
  // Updates recording info with new values
  void updateInfo(const PodInfo& info);

  Parameter<FixedVector<Handle<Receiver>, kMaxComponents>> receivers_;
  Parameter<Handle<Receiver>> enable_;
  Parameter<Handle<Transmitter>> status_;
  Parameter<Handle<EntitySerializer>> entity_serializer_;
  Parameter<Handle<SerializationBuffer>> serialization_buffer_;
  Parameter<Handle<File>> file_;
  Parameter<bool> flush_on_tick_;

  // Recording info
  PodInfo info_;
  // Table that maps a receiver to channel info
  std::unordered_map<gxf_uid_t, ChannelInfo> channel_map_;
  // Pod writer
  pod::Writer writer_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_DW_POD_RECORDER_HPP_
