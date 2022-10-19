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
#ifndef NVIDIA_GXF_TEST_EXTENSIONS_TENSOR_COMPARATOR_HPP
#define NVIDIA_GXF_TEST_EXTENSIONS_TENSOR_COMPARATOR_HPP

#include <string>

#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"

namespace nvidia {
namespace gxf {
namespace test {

enum struct CompareTimestamp {
  kPubtimeOnly = 0,
  kAcqtimeOnly = 1,
  kPubtimeAndAcqtime = 2,
  kNone = 3,
};


// Compares two tensor messages for equality
class TensorComparator : public Codelet {
 public:
  TensorComparator() = default;
  ~TensorComparator() = default;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t start() override  { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override { return GXF_SUCCESS; }

 private:
  // Expected message
  Parameter<Handle<Receiver>> expected_;
  // Actual message
  Parameter<Handle<Receiver>> actual_;

  // Timestamp
  Parameter<CompareTimestamp> compare_timestamp_;
};

}  // namespace test


// Custom parameter parser for CompareTimestamp
template <>
struct ParameterParser<test::CompareTimestamp> {
  static Expected<test::CompareTimestamp> Parse(gxf_context_t context, gxf_uid_t component_uid,
                                       const char* key, const YAML::Node& node,
                                       const std::string& prefix) {
    const std::string value = node.as<std::string>();
    if (strcmp(value.c_str(), "PubtimeOnly") == 0) {
      return test::CompareTimestamp::kPubtimeOnly;
    }
    if (strcmp(value.c_str(), "AcqtimeOnly") == 0) {
      return test::CompareTimestamp::kAcqtimeOnly;
    }
    if (strcmp(value.c_str(), "PubtimeAndAcqtime") == 0) {
      return test::CompareTimestamp::kPubtimeAndAcqtime;
    }
    if (strcmp(value.c_str(), "None") == 0) {
      return test::CompareTimestamp::kNone;
    }
    return nvidia::gxf::Unexpected{GXF_ARGUMENT_OUT_OF_RANGE};
  }
};


}  // namespace gxf
}  // namespace nvidia

#endif
