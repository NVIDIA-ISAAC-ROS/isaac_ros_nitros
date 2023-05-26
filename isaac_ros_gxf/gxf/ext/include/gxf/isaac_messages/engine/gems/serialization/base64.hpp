/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/core/image/image.hpp"

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
