/*
 * SPDX-FileCopyrightText: Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_PY_CODELET_HPP_
#define NVIDIA_GXF_PY_CODELET_HPP_

#include <string>
#include <vector>

#include "gxf/cuda/cuda_common.hpp"
#include "gxf/cuda/cuda_stream_pool.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/clock.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"
#include "pybind11/embed.h"

namespace nvidia {
namespace gxf {

/// C++ side bridge of python codelet. This is the type which all python codelets register.
/// Mandatory params - codelet_name_, codelet_filepath_
/// Optional params - transmitters_, receivers_, allocators_, cuda_stream_pools_, codelet_params_
class __attribute__((visibility("hidden"))) PyCodeletV0 : public Codelet {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;

  /// gets a vector to the allocators which is implicitly converted to
  /// a python list of gxf.std.Allocators when called from python.
  /// @param none
  /// @return vector of allocators
  std::vector<Allocator*> getAllocators();

  /// gets a pointer to nvidia::gxf::clock which is implicitly converted to
  /// a gxf.std.Clock when called from python.
  /// @param name name of the clock component
  /// @return vector of allocators
  Clock* getClock(const std::string name);

  /// gets a vector to the CudaStreamPool pointers which is implicitly converted to
  /// a python list of gxf.cuda.CudaStreamPool objects when called from python.
  /// @param none
  /// @return vector of CudaStreamPool pointers
  std::vector<CudaStreamPool*> getCudaStreamPools();

  /// gets a string to the codelet_params which is converted to python string
  /// when called from python.
  /// @param none
  /// @return string containing the codelet_params
  std::string getParams();

  /// gets a vector to the Receiver pointers which is implicitly converted to
  /// a python list of gxf.std.Receiver when called from python.
  /// @param none
  /// @return vector of nvidia::gxf::Receiver pointers
  std::vector<Receiver*> getReceivers();

  /// gets a vector to the Transmitter pointers which is implicitly converted to
  /// a python list of gxf.std.Transmitter when called from python.
  /// @param none
  /// @return vector of nvidia::gxf::Transmitter
  std::vector<Transmitter*> getTransmitters();

 private:
  Parameter<std::string> codelet_name_;
  Parameter<std::string> codelet_filepath_;
  Parameter<std::vector<Handle<Receiver>>> receivers_;
  Parameter<std::vector<Handle<Transmitter>>> transmitters_;
  Parameter<std::vector<Handle<Allocator>>> allocators_;
  Parameter<std::vector<Handle<CudaStreamPool>>> cuda_stream_pools_;
  Parameter<std::string> codelet_params_;

  template <typename S>
  Expected<Handle<S>> getHandle(const char* name);
  pybind11::object pycodelet;
};
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_PY_CODELET_HPP_
