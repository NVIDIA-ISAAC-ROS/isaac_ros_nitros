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

#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/core/tensor/tensor.hpp"

namespace nvidia {
namespace isaac {

// Maps the unit interval [0,1[ to a 8-bit unsigned integer clamping the value if out of bounds
// This is the inverse function of MapUint8ToUnit.
inline unsigned char MapUnitToUint8(float x) {
  return static_cast<unsigned char>(Clamp(static_cast<int>(255.0f*x + 0.5f), 0, 255));
}

// Maps an 8-bit unsigned integer to the unit interval [0,1[
// This is the inverse function of MapUnitToUint8.
inline float MapUint8ToUnit(unsigned char c) {
  return static_cast<float>(c) / 255.0f;
}

// Converts a 3f pixel to a 3ub pixel by mapping [0,1] to [0,255]
inline Pixel3ub ToPixel3ub(const Pixel3f& color) {
  return Pixel3ub{MapUnitToUint8(color[0]), MapUnitToUint8(color[1]), MapUnitToUint8(color[2])};
}

// Combines a color with an alpha value.
inline Pixel4ub WithAlpha(const Pixel3ub& color, uint8_t alpha) {
  return Pixel4ub{color[0], color[1], color[2], alpha};
}

// Converts a 3ub pixel to a 3f pixel by mapping [0,255] to [0,1]
inline Pixel3f ToPixel3f(const Pixel3ub& color) {
  return Pixel3f{MapUint8ToUnit(color[0]), MapUint8ToUnit(color[1]), MapUint8ToUnit(color[2])};
}

// Interpolates a color
inline Pixel3f Interpolate(const float p, const Pixel3f& a, const Pixel3f& b) {
  const float p0 = 1.0f - p;
  return Pixel3f{p0*a[0] + p*b[0], p0*a[1] + p*b[1], p0*a[2] + p*b[2]};
}

// Converts a color to hex representation, e.g. (255,128,0) -> "#ff8000"
std::string ToHexString(const Pixel3ub& color);
std::string ToHexString(const Pixel3ub& color, unsigned char alpha);

// Some special colors
struct Colors {
  static Pixel3ub Black()        { return Pixel3ub{0,     0,   0}; }
  static Pixel3ub White()        { return Pixel3ub{255, 255, 255}; }
  static Pixel3ub Pink()         { return Pixel3ub{255,  20, 147}; }
  static Pixel3ub NvidiaGreen()  { return Pixel3ub{118, 185,   0}; }
  static Pixel3ub NvidiaGreen2() { return Pixel3ub{  0,  72,  49}; }
  static Pixel3ub NvidiaGreen3() { return Pixel3ub{  0, 132, 113}; }
  static Pixel3ub Red()          { return Pixel3ub{255,   0,   0}; }
  static Pixel3ub Orange()       { return Pixel3ub{255, 128,   0}; }
  static Pixel3ub Yellow()       { return Pixel3ub{255, 255,   0}; }
  static Pixel3ub Green()        { return Pixel3ub{0,   255,   0}; }
  static Pixel3ub Cyan()         { return Pixel3ub{0,   255, 255}; }
  static Pixel3ub Blue()         { return Pixel3ub{0,     0, 255}; }
  static Pixel3ub Magenta()      { return Pixel3ub{255,   0, 255}; }
};

// A color gradient which can be used to interpolate a color based on a floating point number.
class ColorGradient {
 public:
  // Creates a black -> white gradient
  ColorGradient() {
    setColors({Colors::Black(), Colors::White()});
  }
  // Creates a gradient using the given colors
  ColorGradient(const std::vector<Pixel3ub>& colors) {
    setColors(colors);
  }
  ColorGradient(const std::initializer_list<Pixel3ub>& colors) {
    setColors(colors);
  }
  // Creates a gradient using the given colors
  ColorGradient(const std::vector<Pixel3f>& colors)
  : colors_(std::move(colors)) { }
  ColorGradient(std::initializer_list<Pixel3f> colors)
  : colors_(colors.begin(), colors.end()) { }

  // Sets the colors of the gradient
  void setColors(const std::vector<Pixel3ub>& colors) {
    setColors(colors.begin(), colors.end());
  }
  void setColors(const std::initializer_list<Pixel3ub>& colors) {
    setColors(colors.begin(), colors.end());
  }
  template <typename It>
  void setColors(It begin, It end);

  // Get a color on the gradient for a value p between 0 and 1. p will be clamped if out of bounds.
  // If the gradient is empty pink will be returned.
  Pixel3ub operator()(float p) const;

 private:
  std::vector<Pixel3f> colors_;
};

// Greyscale black to white color gradient
ColorGradient BlackWhiteColorGradient();
// Color gradient inspired by "The Starry Night" painting
ColorGradient StarryNightColorGradient();
// Color gradient to match the one above on positive value and extend it in the negative value
ColorGradient StarryNightExtendedColorGradient();
// Color gradient going from black to Nvidia green
ColorGradient BlackGreenColorGradient();
// All rainbow colors red to magenta
ColorGradient RainbowColorGradient();
// From white to orange to black
ColorGradient ErrorColorGradient();
// A gradient to visualize distances (from black to brown to white to light blue)
ColorGradient DistanceColorGradient();
// A gradient going from red to blue over broken white
ColorGradient RedBlueColorGradient();

// Colorizes an image of floats using a color gradient
void Colorize(const ImageConstView1f& input, const ColorGradient& gradient, float min, float max,
              Image3ub& colored);
// Colorizes an image of doubles using a color gradient
void Colorize(const ImageConstView1d& input, const ColorGradient& gradient, double min, double max,
              Image3ub& colored);
// Colorizes a tensor of floats using a color gradient
void Colorize(const CpuTensorConstView2f& input, const ColorGradient& gradient, float min, float max,
              Image3ub& colored);
// Colorizes a tensor of doubles using a color gradient
void Colorize(const CpuTensorConstView2d& input, const ColorGradient& gradient, double min, double max,
              Image3ub& colored);
// Colorizes an image of floats using a color gradient. This asserts if the input and colored image
// dimensions (rows/columns) do not match
void Colorize(const ImageConstView1f& input, const ColorGradient& gradient, float min, float max,
              ImageView3ub& colored);
// Colorizes an image view of doubles using a color gradient. This asserts if the input and
// colored image dimensions (rows/columns) do not match
void Colorize(const ImageConstView1d& input, const ColorGradient& gradient, double min, double max,
              ImageView3ub& colored);
// Colorizes a tensor of floats using a color gradient. This asserts if the input and colored
// image dimensions (rows/columns) do not match
void Colorize(const CpuTensorConstView2f& input, const ColorGradient& gradient, float min, float max,
              ImageView3ub& colored);
// Colorizes a tensor of doubles using a color gradient. This asserts if the input and colored
// image dimensions (rows/columns) do not match
void Colorize(const CpuTensorConstView2d& input, const ColorGradient& gradient, double min, double max,
              ImageView3ub& colored);

// A list of colors which can be used to get a color based on an integer
class IndexedColors {
 public:
  // Creates a list with two colors: black and white
  IndexedColors() : colors_({Pixel3ub::Zero(), Pixel3ub{255, 255, 255}}) { }

  // Initializes the object with the given list of colors
  IndexedColors(std::initializer_list<Vector3ub> colors) : colors_(colors) { }

  // Creates a list of random colors
  IndexedColors(size_t count, size_t seed) : colors_(count) {
    colors_[0] = Pixel3ub::Zero();
    std::default_random_engine rng(seed);
    std::uniform_int_distribution<uint8_t> rnd(0, 255);
    for (size_t i = 1; i < colors_.size(); i++) {
      colors_[i] = Pixel3ub{rnd(rng), rnd(rng), rnd(rng)};
    }
  }

  // Retrieves the color for the given index. If there are not enough colors the index is wrapped.
  const Pixel3ub& operator()(int index) const {
    const int value = index % colors_.size();
    return colors_[value >= 0 ? value : (value + colors_.size())];
  }

 private:
  std::vector<Pixel3ub> colors_;
};

// 256 random colors using a default seed
IndexedColors Random256IndexedColors();

// 22 distinct colors after Sasha Trubetskoy
IndexedColors Distinct22IndexedColors();

// Colorizes an image of indices using a list of colors
template <typename Index>
void ColorizeIndices(const ImageConstView<Index, 1>& indices, const IndexedColors& colors,
                     Image3ub& result) {
  const size_t rows = indices.rows();
  const size_t cols = indices.cols();
  result.resize(indices.dimensions());
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      result(row, col) = colors(indices(row, col));
    }
  }
}

// Colorizes an image of indices using a list of colors. This asserts if the input and colored
// image dimensions (rows/columns) do not match
template <typename Index>
void ColorizeIndices(const ImageConstView<Index, 1>& indices, const IndexedColors& colors,
                     ImageView3ub& result) {
  ASSERT(result.rows() == indices.rows() && result.cols() == indices.cols(),
         "Output dimension (%d, %d) != Input dimension (%d, %d)", result.rows(), result.cols(),
         indices.rows(), indices.cols());

  const size_t rows = indices.rows();
  const size_t cols = indices.cols();
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      result(row, col) = colors(indices(row, col));
    }
  }
}
// -------------------------------------------------------------------------------------------------

template <typename It>
void ColorGradient::setColors(It begin, It end) {
  colors_.clear();
  colors_.reserve(std::distance(begin, end));
  for (; begin != end; ++begin) {
    colors_.push_back(ToPixel3f(*begin));
  }
}

inline Pixel3ub ColorGradient::operator()(float p) const {
  if (colors_.empty()) {
    return Colors::Pink();
  }
  const float pn = static_cast<float>(colors_.size() - 1) * p;
  const int pi = std::floor(pn);
  if (pi < 0) {
    return ToPixel3ub(colors_.front());
  }
  if (static_cast<size_t>(pi + 1) >= colors_.size()) {
    return ToPixel3ub(colors_.back());
  }
  const float pr = pn - static_cast<float>(pi);
  return ToPixel3ub(Interpolate(pr, colors_[pi], colors_[pi + 1]));
}

// Converts a pixel from RGB scheme to HSV
// Reference for conversion: https://en.wikipedia.org/wiki/HSL_and_HSV
void ConvertRGBToHSV(PixelRef3ub pixel);

// Converts a pixel from HSV scheme to RGB
// Reference for conversion: https://en.wikipedia.org/wiki/HSL_and_HSV
void ConvertHSVToRGB(PixelRef3ub pixel);

}  // namespace isaac
}  // namespace nvidia
