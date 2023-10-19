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
#ifndef DETECTNET__DETECTION2_D_HPP_
#define DETECTNET__DETECTION2_D_HPP_

#include <string>
#include <vector>

namespace nvidia
{
namespace isaac_ros
{

struct Hypothesis
{
  // The unique ID of the object class
  std::string class_id;
  // The probability or confidence value of the detected object
  double score;
};

// Data structure holding meta information about detections
// based on ros2 msg vision_msgs::Detection2DArray
// https://github.com/ros-perception/vision_msgs/blob/ros2/msg/Detection2DArray.msg
struct Detection2D
{
  // The origin is the top left corner of the image
  // The postive x axis is towards the right from the origin
  // The postive y axis is towards the bottom from the origin
  // The 2D position (in pixels) and orientation of the bounding box center
  double center_x;
  double center_y;
  // The total size (in pixels) of the bounding box
  double size_x;
  double size_y;
  // A vector of object classes and corresponding confidence values
  // for the bounding box specified in this struct
  std::vector<Hypothesis> results;
};

}  // namespace isaac_ros
}  // namespace nvidia

#endif  // DETECTNET__DETECTION2_D_HPP_
