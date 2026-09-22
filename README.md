# Isaac ROS NITROS

NVIDIA Isaac Transport for ROS package for hardware-acceleration friendly movement of messages.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros/image5-1.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/release-5.0/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros/image5-1.gif/" width="600px"/></a></div>

## Overview

**NITROS is deprecated.** Isaac ROS nodes now use ROS 2 messages with `rosidl::Buffer` fields and
the CUDA buffer backend, which are native ROS 2 Lyrical features. NITROS type adaptation and
negotiation, Managed NITROS, CUDA with NITROS, PyNITROS, and the NITROS Bridge APIs will be removed
in a future Isaac ROS release. Node-level compatibility is maintained, but applications that call
NITROS APIs directly require source-level migration. Refer to
[From NITROS to rosidl::Buffer](https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/nitros_migration.html) for the migration
guide, and [rosidl::Buffer and Buffer Backends](https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/index.html) for the
replacement architecture.

[Isaac ROS NITROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros) now contains the vendor
packages that distribute NVIDIA’s precompiled SDKs to other Isaac ROS repositories, along with the
remaining NITROS Bridge converter.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io) to learn how to use
this repository.

---

## Packages

* `cuapriltags_vendor`: Vendor package for the precompiled NVIDIA cuAprilTags SDK.
* `cumotion_vendor`: Vendor package for the precompiled NVIDIA cuMotion SDK.
* `cuvslam_vendor`: Vendor package for the precompiled NVIDIA cuVSLAM SDK.
* `isaac_ros_nitros_bridge_ros2`: Converter between NITROS bridge messages and ROS 2 messages.

## Latest

Update 2026-09-21: Deprecated NITROS in favor of rosidl::Buffer and the CUDA buffer backend
