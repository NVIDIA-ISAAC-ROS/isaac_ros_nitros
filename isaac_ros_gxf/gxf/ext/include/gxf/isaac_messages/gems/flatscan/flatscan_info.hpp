// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace nvidia {
namespace isaac {

// Data structure holding meta information about flatscan messages. Every flatscan message must
// contain an instance of this message.
struct FlatscanInfo {
  // If the beam free range end is greater than or equal to this value the beam did not hit an
  // obstacle at the end of the free range.
  double out_of_range;
  // Horizontal width of every beam in radians.
  double beam_width;
  // The lower z value of the height slice.
  double height_min;
  // The upper z value of the height slice.
  double height_max;
};

}  // namespace isaac
}  // namespace nvidia
