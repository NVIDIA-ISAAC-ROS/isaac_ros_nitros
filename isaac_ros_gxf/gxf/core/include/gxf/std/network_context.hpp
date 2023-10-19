// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef GXF_STD_NETWORK_CONTEXT_HPP
#define GXF_STD_NETWORK_CONTEXT_HPP

#include "gxf/core/component.hpp"
#include "gxf/core/handle.hpp"

namespace nvidia {
namespace gxf {

class NetworkContext : public Component {
 public:
  // Finds transmitters and receivers passes the network context to transmitter
  // and receivers and make connection between them
  virtual Expected<void> addRoutes(const Entity& entity) = 0;

  // Closes the connection between transmitters and receivers
  virtual Expected<void> removeRoutes(const Entity& entity) = 0;
};

}  // namespace gxf
}  // namespace nvidia

#endif
