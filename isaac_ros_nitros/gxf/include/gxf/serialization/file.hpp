/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_SERIALIZATION_FILE_HPP_
#define NVIDIA_GXF_SERIALIZATION_FILE_HPP_

#include <ctime>
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
  // Updates the file timestamp using the current time
  void updateTimestamp();

 private:
  Parameter<Handle<Allocator>> allocator_;
  Parameter<std::string> file_path_;
  Parameter<std::string> file_mode_;
  Parameter<size_t> buffer_size_;
  Parameter<bool> timestamp_;
  Parameter<bool> write_protect_;

  // File stream
  std::FILE* file_;
  // Stream buffer
  MemoryBuffer buffer_;
  // Timestamp of when the file was opened
  std::time_t opened_;
  // Mutex to guard concurrent access
  mutable std::recursive_mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_FILE_HPP_
