// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_HTTP_HTTP_CLIENT_HPP_
#define NVIDIA_GXF_HTTP_HTTP_CLIENT_HPP_

#include <string>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

/// @brief http client that can be used by owner gxf component to send request to any http server
//
// Generic http client interface
class HttpClient : public Component {
 public:
  virtual ~HttpClient() = default;

  typedef struct {
    int status_code;
    std::string body;
  } Response;

  /// @brief standard Http GET request
  // uri is "{path_name}/{resource_name}"
  virtual Expected<nvidia::gxf::HttpClient::Response> getRequest(const std::string& uri) = 0;

  /// @brief standard Http POST request
  // uri is "{path_name}/{resource_name}"
  virtual Expected<nvidia::gxf::HttpClient::Response>
  postRequest(const std::string& uri,
              const std::string& payload,
              const std::string& content_type) = 0;
};

}  // namespace gxf
}  // namespace nvidia

#endif
