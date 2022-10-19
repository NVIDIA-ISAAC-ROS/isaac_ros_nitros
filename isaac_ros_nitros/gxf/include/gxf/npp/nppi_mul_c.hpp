/*
 * SPDX-FileCopyrightText: Copyright (c) 2020 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_NPP_NPPI_MUL_C_HPP
#define NVIDIA_GXF_NPP_NPPI_MUL_C_HPP

#include <vector>

#include "gxf/std/allocator.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/parameter_parser_std.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace gxf {

// Multiplies a CUDA tensor with a constant factor using NPP.
class NppiMulC : public Codelet {
 public:
  virtual ~NppiMulC() = default;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  Parameter<Handle<Receiver>> in_;
  Parameter<std::vector<double>> factor_;
  Parameter<Handle<Allocator>> pool_;
  Parameter<Handle<Transmitter>> out_;
};

}  // namespace gxf
}  // namespace nvidia

#endif
