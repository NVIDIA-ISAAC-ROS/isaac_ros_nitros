/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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
