/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_SERIALIZATION_ENTITY_RECORDER_HPP_
#define NVIDIA_GXF_SERIALIZATION_ENTITY_RECORDER_HPP_

#include <string>

#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/serialization/file_stream.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace gxf {

// Records incoming entities by serializaing and writing to a file.
// Uses one file to store binary data and a second file as an index to enable random-access.
class EntityRecorder : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  Parameter<Handle<Receiver>> receiver_;
  Parameter<Handle<EntitySerializer>> entity_serializer_;
  Parameter<std::string> directory_;
  Parameter<std::string> basename_;
  Parameter<bool> flush_on_tick_;

  // File stream for data index
  FileStream index_file_stream_;
  // File stream for binary data
  FileStream binary_file_stream_;
  // Offset into binary file
  size_t binary_file_offset_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_ENTITY_RECORDER_HPP_
