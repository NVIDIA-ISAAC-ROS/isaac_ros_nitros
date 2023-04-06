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
#ifndef NVIDIA_GXF_SERIALIZATION_FILE_HPP_
#define NVIDIA_GXF_SERIALIZATION_FILE_HPP_

#include <mutex>
#include <string>

#include "gxf/serialization/endpoint.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/memory_buffer.hpp"

namespace nvidia {
namespace gxf {

// Wrapper around C file I/O API
class File : public Endpoint {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t write_abi(const void* data, size_t size, size_t* bytes_written) override;
  gxf_result_t read_abi(void* data, size_t size, size_t* bytes_read) override;

  // Opens a file with the given mode
  // Uses parameter values if arguments are not specified
  Expected<void> open(const char* path = nullptr, const char* mode = nullptr);
  // Closes the opened file
  Expected<void> close();
  // Clears the end-of-file and error indicators
  void clear();
  // Returns true if the end-of-file indicator is set
  bool eof();
  // Returns true if the error indicator is set
  bool error();
  // Flushes the output buffer of the file stream
  Expected<void> flush();
  // Sets the file position to the given offset
  Expected<void> seek(size_t offset);
  // Returns the current file position
  Expected<size_t> tell();
  // Returns true if the file is open
  bool isOpen();
  // Returns the configured file path
  const char* path();
  // Returns the configured file mode
  const char* mode();
  // Renames the file
  Expected<void> rename(const char* path);
  // Prepends a timestamp to the file name
  // Uses local time zone by default and UTC if specified
  Expected<void> addTimestamp(int64_t timestamp, bool utc = false);
  // Write-protects the file
  Expected<void> writeProtect();

 private:
  Parameter<Handle<Allocator>> allocator_;
  Parameter<std::string> file_path_;
  Parameter<std::string> file_mode_;
  Parameter<size_t> buffer_size_;

  // File stream
  std::FILE* file_;
  // Stream buffer
  MemoryBuffer buffer_;
  // Mutex to guard concurrent access
  mutable std::recursive_mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_FILE_HPP_
