/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {

// Copies an image
template <typename K, int N, typename SourceContainer, typename TargetContainer>
void Copy(const ImageBase<K, N, SourceContainer>& source, ImageBase<K, N, TargetContainer>& target);

// Sets all pixels of an image to the given value
template <typename K, int N, typename Container>
void FillPixels(ImageBase<K, N, Container>& image, const Pixel<K, N>& value);

// Sets all elements of all pixels to the given `value`
template <typename K, int N, typename Container>
void FillElements(ImageBase<K, N, Container>& image, K value);

// Sets all elements of all pixels to 0
template <typename K, int N, typename Container>
void FillElementsWithZero(ImageBase<K, N, Container>& image) {
  FillElements(image, K(0));
}

// Resize down an image by a factor N.
template <int Factor, typename K, int N, typename Container>
Image<K, N> Reduce(const ImageBase<K, N, Container>& img);
template <typename K, int N, typename Container>
Image<K, N> Reduce(const ImageBase<K, N, Container>& img, int factor);

// Enlarge an image by a factor N
template <int NRows, int NCols = NRows, class Image>
Image Enlarge(const Image& img);

// Convert an image from a format to another given a pixel conversion function
template <typename Out, typename In, typename F>
Out Convert(const In& img, F convert);
template <typename Out, typename In, typename F>
void Convert(const In& img, Out& out, F convert);

// Normalizes an image
template <typename K, typename Container>
void Normalize(const ImageBase<K, 1, Container>& input, Image1ub& output);
template <typename K, typename Container>
void Normalize(const ImageBase<K, 1, Container>& input, K min, K max, Image1ub& output);

// Crops the image
template <typename K, int N, typename Container>
void Crop(const ImageBase<K, N, Container>& img, const Vector2i& crop_start,
          const Vector2i& crop_size, Image<K, N>& dest_img);

// Convert disparity to depth using baseline and focal length.
// The disparity_img is expected to be in pixels
// The depth will have same unit as baseline. The baseline, min_depth and max_depth should all be
// in the same units.
template <typename K, typename Container,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type = 0>
void ConvertDisparityToDepth(const ImageBase<K, 1, Container>& disparity_img, K baseline,
                             K focal_length_px, K min_depth, K max_depth, Image<K, 1>& depth_img);

// Stitches two images (with same number of rows) together side by side
void JoinTwoImagesSideBySide(const ImageConstView3ub& left_image,
                             const ImageConstView3ub& right_image, Image3ub& joint_image);

// Splits an image into two halfs
void SplitImages(const ImageConstView3ub& joint, Image3ub& left, Image3ub& right);

// -------------------------------------------------------------------------------------------------

template <typename K, int N, typename SourceContainer, typename TargetContainer>
void Copy(const ImageBase<K, N, SourceContainer>& source,
          ImageBase<K, N, TargetContainer>& target) {
  // Asserts that images have the same shape
  ASSERT(source.rows() == target.rows(), "row count mismatch: %zu vs %zu",
         source.rows(), target.rows());
  ASSERT(source.cols() == target.cols(), "col count mismatch: %zu vs %zu",
         source.cols(), target.cols());
  // Copy the bytes
  CopyMatrixRaw(reinterpret_cast<const void*>(source.element_wise_begin()),
                source.getStride(),
                BufferTraits<SourceContainer>::kStorageMode,
                reinterpret_cast<void*>(target.element_wise_begin()),
                target.getStride(),
                BufferTraits<TargetContainer>::kStorageMode,
                source.rows(), source.cols() * N * sizeof(K));
}

template <typename K, int N, typename Container>
void FillPixels(ImageBase<K, N, Container>& image, const Pixel<K, N>& pixel) {
  for (int row = 0; row < image.rows(); row++) {
    for (int col = 0; col < image.cols(); col++) {
      image(row, col) = pixel;
    }
  }
}

template <typename K, typename Container>
void FillPixels(ImageBase<K, 1, Container>& image, K pixel) {
  // Use the faster FillElements for 1-channel images
  FillElements(image, pixel);
}

template <typename K, int N, typename Container>
void FillElements(ImageBase<K, N, Container>& image, K value) {
  const int elements_per_row = image.channels() * image.cols();
  for (int row = 0; row < image.rows(); row++) {
    K* row_pointer = image.row_pointer(row);
    std::fill(row_pointer, row_pointer + elements_per_row, value);
  }
}

template <int Factor, typename K, int N, typename Container>
Image<K, N> Reduce(const ImageBase<K, N, Container>& img) {
  Image<K, N> out(img.rows() / Factor, img.cols() / Factor);
  for (int row = 0; row < out.rows(); row++) {
    for (int col = 0; col < out.cols(); col++) {
      out(row, col) = img(Factor * row, Factor * col);
    }
  }
  return out;
}

template <typename K, int N, typename Container>
Image<K, N> Reduce(const ImageBase<K, N, Container>& img, int factor) {
  Image<K, N> out(img.rows() / factor, img.cols() / factor);
  for (int row = 0; row < out.rows(); row++) {
    for (int col = 0; col < out.cols(); col++) {
      out(row, col) = img(factor * row, factor * col);
    }
  }
  return out;
}

template <int NRows, int NCols, class Image>
Image Enlarge(const Image& img) {
  Image out(img.rows() * NRows, img.cols() * NCols);
  for (int row = 0; row < out.rows(); row++) {
    for (int col = 0; col < out.cols(); col++) {
      out(row, col) = img(row / NRows, col / NCols);
    }
  }
  return out;
}

template <class Out, class In, typename F>
Out Convert(const In& img, F convert) {
  Out out(img.rows(), img.cols());
  for (int pixel = 0; pixel < out.num_pixels(); pixel++) {
    out[pixel] = convert(img[pixel]);
  }
  return out;
}

template <class Out, class In, typename F>
void Convert(const In& img, Out& out, F convert) {
  ASSERT(out.dimensions() == img.dimensions(), "dimensions mismatch");
  for (int pixel = 0; pixel < out.num_pixels(); pixel++) {
    out[pixel] = convert(img[pixel]);
  }
}

template <typename K, typename Container>
void Normalize(const ImageBase<K, 1, Container>& input, Image1ub& output) {
  K min = input[0];
  K max = input[0];
  for (int pixel = 0; pixel < input.num_pixels(); pixel++) {
    min = std::min(min, input[pixel]);
    max = std::max(max, input[pixel]);
  }
  if (min == max) {
    min -= K(1.0);
    max += K(1.0);
  }
  output = Convert<Image1ub>(
      input, [&](K val) { return static_cast<uint8_t>(K(255.9) * (val - min) / (max - min)); });
}

template <typename K, typename Container>
void Normalize(const ImageBase<K, 1, Container>& input, K min, K max, Image1ub& output) {
  ASSERT(min < max, "Invalid range");
  Convert<Image1ub>(input, output, [&](K val) {
    return static_cast<uint8_t>(K(255.9) * Clamp01((val - min) / (max - min)));
  });
}

template <typename K, int N, typename Container>
void Crop(const ImageBase<K, N, Container>& img, const Vector2i& crop_start,
          const Vector2i& crop_size, Image<K, N>& dest_img) {
  ASSERT((crop_start.array() >= 0).all(), "Invalid crop start location.");
  ASSERT((crop_size.array() > 0).all(), "Invalid crop size.");
  Vector2i img_size{img.rows(), img.cols()};
  ASSERT(((crop_start + crop_size).array() <= img_size.array()).all(), "Invalid crop size");

  EigenImageConstMap<K> eigen_img(img.element_wise_begin(), img.rows(), img.cols() * N);
  dest_img.resize(crop_size[0], crop_size[1]);
  EigenImageMap<K> eigen_dest_img(dest_img.element_wise_begin(), dest_img.rows(),
                                  dest_img.cols() * N);
  eigen_dest_img = eigen_img.block(crop_start[0], crop_start[1] * N, crop_size[0],
                                   crop_size[1] * N);
}

}  // namespace isaac
