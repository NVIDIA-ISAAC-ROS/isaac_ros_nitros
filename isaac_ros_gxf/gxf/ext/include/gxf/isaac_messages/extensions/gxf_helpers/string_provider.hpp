// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <string>

#include "gxf/core/component.hpp"

namespace nvidia {
namespace isaac {

// A component that provides access to a string. This is useful for situations where one would like
// to share the same string between multiple components, for example to implement a namespace/prefix
// for a frame name of a pose tree frame.
class StringProvider : public gxf::Component {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;

  // Gets the value of the string.
  std::string value() const;

 private:
  gxf::Parameter<gxf::Handle<StringProvider>> prefix_;
  gxf::Parameter<gxf::Handle<StringProvider>> suffix_;
  gxf::Parameter<std::string> value_;
};

}  // namespace isaac
}  // namespace nvidia
