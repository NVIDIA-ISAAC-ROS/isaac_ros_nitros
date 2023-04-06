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
#ifndef NVIDIA_GXF_SERIALIZATION_FILE_STREAM_HPP_
#define NVIDIA_GXF_SERIALIZATION_FILE_STREAM_HPP_

#include <fstream>
#include <string>
#include <utility>

#include "gxf/serialization/endpoint.hpp"

namespace nvidia {
namespace gxf {

#pragma pack(push, 1)
struct EntityIndex {
  uint64_t log_time;     // Time when data was logged
  uint64_t data_size;    // Size of data block
  uint64_t data_offset;  // Location of data block
};
#pragma pack(pop)

// Graph endpoint that exchanges data using files
// Input file path is used for reading (empty string to disable reading)
// Output file path is used for writing (empty string to disable writing)
// The same file can be used for both reading and writing
class FileStream : public Endpoint {
 public:
  // Extension for index files
  static constexpr const char* kIndexFileExtension = ".gxf_index";
  // Extension for binary files
  static constexpr const char* kBinaryFileExtension = ".gxf_entities";

  FileStream(std::string input_file, std::string output_file)
    : input_file_path_{input_file}, output_file_path_{output_file} {}
  FileStream() = default;
  ~FileStream() = default;
  FileStream(const FileStream& other) = delete;
  FileStream(FileStream&& other) { *this = std::move(other); }
  FileStream& operator=(const FileStream& other) = delete;
  FileStream& operator=(FileStream&& other) {
    input_file_path_ = std::move(other.input_file_path_);
    output_file_path_ = std::move(other.output_file_path_);
    input_file_ = std::move(other.input_file_);
    output_file_ = std::move(other.output_file_);
    return *this;
  }

  Expected<void> open();
  Expected<void> close();

  gxf_result_t write_abi(const void* data, size_t size, size_t* bytes_written) override;
  gxf_result_t read_abi(void* data, size_t size, size_t* bytes_read) override;

  // Clears error state flags for input and output streams
  void clear();
  // Flushes output stream buffer to file
  // Returns an error if operation failed
  Expected<void> flush();
  // Moves the output stream to the desired position
  Expected<void> setWriteOffset(size_t index);
  // Returns the current position of the output stream
  Expected<size_t> getWriteOffset();
  // Moves the input stream to the desired position
  Expected<void> setReadOffset(size_t index);
  // Returns the current position of the input stream
  Expected<size_t> getReadOffset();

 private:
  // Path to input file
  std::string input_file_path_;
  // Path to output file
  std::string output_file_path_;
  // Input file stream
  std::ifstream input_file_;
  // Output file stream
  std::ofstream output_file_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_FILE_STREAM_HPP_
