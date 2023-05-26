/*
Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/algorithm/string_utils.hpp"

namespace isaac {

// Gets the dimensions of an image. The dimensions are written to `shape` in the order:
// rows (0), cols (1), channels (2).
bool LoadImageShape(const std::string& filename, Vector3i& shape);

// Loads an image from a PNG file. The function will return false if the PNG could not be loaded.
// This can for example happen if the filename is invalid or if the PNG doesn't have the requested
// format.
bool LoadPng(const std::string& filename, Image1ub& image);
bool LoadPng(const std::string& filename, Image1ui16& image);
bool LoadPng(const std::string& filename, Image3ub& image);
bool LoadPng(const std::string& filename, Image4ub& image);
bool LoadPng(const std::string& filename, ImageView1ub& tensor);
bool LoadPng(const std::string& filename, ImageView3ub& tensor);
bool LoadPng(const std::string& filename, CpuTensorView2ub& tensor);

// Loads an image from a JPEG file. The function will return false if the JPEG could not be loaded.
// This can for example happen if the filename is invalid or if the JPEG doesn't have the requested
// format.
bool LoadJpeg(const std::string& filename, Image1ub& image);
bool LoadJpeg(const std::string& filename, Image3ub& image);
bool LoadJpeg(const std::string& filename, ImageView1ub& image);
bool LoadJpeg(const std::string& filename, ImageView3ub& image);

// Loads an image from JPEG file or PNG file. The function will return false if the image could not
// be loaded.
bool LoadImage(const std::string& filename, Image1ub& image);
bool LoadImage(const std::string& filename, Image3ub& image);
bool LoadImage(const std::string& filename, ImageView1ub& image);
bool LoadImage(const std::string& filename, ImageView3ub& image);

// Saves an image to a file in the PNG format. The function will return false if the PNG could not
// saved. This might for example happen if the file could not be opened.
bool SavePng(const ImageConstView1ub& image, const std::string& filename);
bool SavePng(const ImageConstView3ub& image, const std::string& filename);
bool SavePng(const ImageConstView4ub& image, const std::string& filename);
bool SavePng(const ImageConstView1ui16& image, const std::string& filename);

// Saves an image to a file in the JPEG format. The function will return false if the JPEG could not
// saved. This might for example happen, if the file could not be opened. Quality of 100 means
// nearly lossless, but slow to encode and result in bigger images. While quality of 1 is fast
// and produce very small image with a very low quality. (Note: 75 seems a good compromise)
bool SaveJpeg(const ImageConstView1ub& image, const std::string& filename, const int quality = 75);
bool SaveJpeg(const ImageConstView3ub& image, const std::string& filename, const int quality = 75);

// Encodes an image into a buffer in the PNG format
void EncodePng(const ImageConstView1ub& image, std::vector<uint8_t>& encoded);
void EncodePng(const ImageConstView3ub& image, std::vector<uint8_t>& encoded);
void EncodePng(const ImageConstView4ub& image, std::vector<uint8_t>& encoded);

// Encodes an image into a file in the JPEG format
// Quality of 100 is nearly lossless but slow to encode and result in bigger images while 1 is fast
// and produce very small image with a very low quality. (Note: 75 seems a good compromise)
void EncodeJpeg(const ImageConstView1ub& image, int quality, std::vector<uint8_t>& encoded);
void EncodeJpeg(const ImageConstView3ub& image, int quality, std::vector<uint8_t>& encoded);

}  // namespace isaac
