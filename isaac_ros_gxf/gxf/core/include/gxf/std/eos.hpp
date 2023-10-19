// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef EOS_HPP
#define EOS_HPP

#include <utility>

#include "gxf/core/entity.hpp"

namespace nvidia {
namespace gxf {

// A component which represents an end-of-stream notification
class EndOfStream {
 public:
  // Returns the stream which generated the notification. Negative value indicates
  // pipeline/graph EoS. Non-negative value indicates individual stream EoS
  int64_t stream_id() { return stream_id_; }

  void stream_id(int64_t stream_id) { stream_id_ = stream_id; }

  EndOfStream() = default;

  ~EndOfStream() = default;

  EndOfStream(const EndOfStream&) = delete;

  EndOfStream(EndOfStream&& other) { *this = std::move(other); }

  EndOfStream& operator=(const EndOfStream&) = delete;

  EndOfStream& operator=(EndOfStream&& other) {
    stream_id_ = other.stream_id_;
    return *this;
  }

  // Factory method to create EOS message entity
  static Expected<Entity> createEoSMessage(gxf_context_t context, int64_t stream_id = -1) {
    Expected<Entity> message = Entity::New(context);
    if (!message) { return message; }

    auto eos = message.value().add<EndOfStream>();
    if (!eos) { return ForwardError(eos); }
    eos.value()->stream_id_ = stream_id;

    return message;
  }

 private:
  int64_t stream_id_ = -1;
};

}  // namespace gxf
}  // namespace nvidia

#endif
