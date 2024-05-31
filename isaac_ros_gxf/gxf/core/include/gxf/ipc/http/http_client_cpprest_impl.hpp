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
#ifndef NVIDIA_GXF_HTTP_CPPREST_HTTP_CLIENT_HPP_
#define NVIDIA_GXF_HTTP_CPPREST_HTTP_CLIENT_HPP_

#include "gxf/ipc/http/http_client.hpp"
#include "cpprest/http_client.h"  // NOLINT

#include <memory>
#include <string>

namespace nvidia {
namespace gxf {

/// @brief CppRestSDK implementation on HttpClient interface
class CppRestHttpClient : public HttpClient {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  Expected<nvidia::gxf::HttpClient::Response> getRequest(const std::string& uri) override;
  Expected<nvidia::gxf::HttpClient::Response> postRequest(const std::string& uri,
                                                          const std::string& payload,
                                                          const std::string& content_type) override;

 private:
  nvidia::gxf::Parameter<std::string> server_ip_port_;
  nvidia::gxf::Parameter<bool> use_https_;
  std::unique_ptr<web::uri> base_uri_;
  std::unique_ptr<web::http::client::http_client> raw_client_;
};

}  // namespace gxf
}  // namespace nvidia

#endif
