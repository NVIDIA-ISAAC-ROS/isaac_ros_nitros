# Isaac ROS NITROS

NVIDIA Isaac Transport for ROS package for hardware-acceleration friendly movement of messages.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros/image5-1.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros/image5-1.gif/" width="600px"/></a></div>

## Overview

[Isaac ROS NITROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros) contains NVIDIA’s implementation
of type adaptation and negotiation in ROS 2. To learn more about NITROS, see [here](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html).

Isaac ROS NITROS is composed of a number of individual packages, each with either a functional or structural purpose:

`isaac_ros_gxf`:
: This package serves as a container for precompiled GXF extensions used by other Isaac ROS packages.
  While a number of GXF extensions used by Isaac ROS are provided with source, the extensions contained in `isaac_ros_gxf` are license constrained and are thus shipped as `.so` binaries.

`isaac_ros_managed_nitros`:
: This package contains the wrapper classes that enable developers to add NITROS-compatible publishers and subscribers to third-party CUDA-based ROS nodes.
  For more information about CUDA with NITROS, see [here](https://nvidia-isaac-ros.github.io/concepts/nitros/cuda_with_nitros.html).

`isaac_ros_nitros`:
: This package contains the base `NitrosNode` class and associated core utilities that serve as the foundation for all NITROS-based ROS nodes.

`isaac_ros_nitros_interfaces`:
: This package contains the definitions of the custom ROS 2 interfaces that facilitate type negotiation between NITROS nodes.

`isaac_ros_nitros_topic_tools`:
: This folder contains a NITROS based implementation of some of the nodes in the [topic_tools package](https://github.com/ros-tooling/topic_tools).

`isaac_ros_nitros_type`:
: This folder contains a number of packages, each defining a specific NITROS type and the associated type adaptation logic to convert to and from a standard ROS type.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_gxf`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_gxf/index.html)
* [`isaac_ros_managed_nitros`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_managed_nitros/index.html)
* [`isaac_ros_nitros`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros/index.html#quickstart)
* [`isaac_ros_nitros_interfaces`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros_interfaces/index.html)
* [`isaac_ros_nitros_topic_tools`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros_topic_tools/index.html)
  * [NitrosCameraDrop Node](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros_topic_tools/index.html#nitroscameradrop-node)
* [`isaac_ros_nitros_type`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros/isaac_ros_nitros_type/index.html)

## Latest

Update 2024-09-26: Update for Isaac ROS 3.1
