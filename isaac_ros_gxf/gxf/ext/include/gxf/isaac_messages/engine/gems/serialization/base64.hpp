// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "engine/core/image/image.hpp"

namespace nvidia {
namespace isaac {
namespace serialization {

// Encodes an Image3ub image into a Bitmap image base 64 encoded ready to be used in HTML code.
// WARNING: The image is converted to BMP format which requires the memory to be 4-alligned at the
// beginning of each row. In order to avoid memory copy and keep the code efficient, the encoded
// image might contain more additional columns filled with random data, it will be the
// responsability of the consumer to ignore these columns.
std::string Base64Encode(const Image1ub& image);
std::string Base64Encode(const Image3ub& image);
// Encodes some data in base 64.
std::string Base64Encode(const uint8_t* bytes_to_encode, size_t in_len);
// Decode a base 64 encoded string into its original format.
std::string Base64Decode(const std::string& encoded_string);

}  // namespace serialization
}  // namespace isaac
}  // namespace nvidia
