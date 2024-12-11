# Isaac ROS NITROS Bridge

Use Isaac ROS packages in your ROS 1 Noetic with NITROS-enabled performance.

<div align="center"><a class="reference internal image-reference" href="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_overview.png/"><img alt="image" src="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_overview.png/" width="800px"/></a></div>

---

## Overview

The [Isaac ROS NITROS Bridge](https://gitlab-master.nvidia.com/isaac_ros/isaac_ros_nitros_bridge) brings
accelerated computing to versions of ROS where it is not supported, by improving
the performance of bridging between ROS versions.

Isaac ROS provides accelerated computing ROS packages for robotics development
in ROS 2 with NITROS; this is enabled with the addition of type adaptation
(REP-2007) and type negotiation (REP-2009) in ROS 2 Humble, which is not
available in previous versions of ROS.

Accelerated computing is essential to move beyond the limits of single threaded
performance on a CPU. To take advantage of this, it may not be practical to migrate to
a new version of ROS 2 for robotics applications that were developed previously. Some
developers have chosen to bridge between their current version of ROS, and
what is used with Isaac ROS. This improves existing robotics applications with the
addition of accelerated computing; unfortunately, the ROS bridge includes a CPU based memory
copy bridge toll.

Isaac ROS NITROS Bridge packages improves performance with a ROS bridge.
The bridge toll is removed by moving data from CPU to GPU to avoid CPU memory copies.
This significantly improves performance across processes in different versions of ROS.
In addition the data in GPU memory can be used in place, by accelerated computing.

<div align="center"><a class="reference internal image-reference" href="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge.png/"><img alt="image" src="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge.png/" width="800px"/></a></div>

Illustrated above is the use of NITROS Converters between ROS Noetic and ROS 2 Humble.
When sending an image from ROS Noetic to ROS 2 Humble,
the NITROS converter moves the image to GPU accelerated memory avoiding CPU memory copies.
On the Humble side of the bridge the NITROS converter adapts the image in GPU memory into
a NITROS image for use with accelerated computing. This also works the other direction from
ROS 2 Humble to ROS Noetic. When sending an NITROS image from Humble to Noetic,
the NITROS Converter maintains the image in GPU accelerated memory avoiding CPU memory
copies over the bridge, moving the image into CPU accessible memory in Noetic.

This same principle can be applied to ROS 2, for example between Foxy and Humble
to take advantage of Isaac ROS in Humble, from applications developed in Foxy
without migrating versions of ROS.

<div align="center"><a class="reference internal image-reference" href="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/ros_bridge.png/"><img alt="image" src="https://gitlab-master.nvidia.com/isaac_ros/nvidia-isaac-ros/-/raw/dev/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/ros_bridge.png/" width="800px"/></a></div>

Illustrated above is the use of the ROS bridge without the benefits of NITROS converters.
Images sent from Noetic to Humble and visa-versa are copies across the ROS processes using
the CPU limiting performance and increasing latency.

By avoiding unnecessary CPU memory copies the Isaac ROS NITROS bridge improves accelerated
computing performance and reduces end to end latency across versions of ROS.

## Performance

Isaac ROS NITROS Bridge can move 1080p images between ROS 1 Noetic and Isaac ROS NITROS packages up to 3x faster than the available [ros1_bridge](https://github.com/ros2/ros1_bridge).
See [Isaac ROS Benchmark](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_benchmark/index.html) scripts for [NITROS bridge](https://gitlab-master.nvidia.com/isaac_ros/isaac_ros_benchmark/-/blob/dev/benchmarks/isaac_ros_nitros_bridge_benchmark/scripts/isaac_ros_nitros_bridge.py) and [ros1_bridge](https://gitlab-master.nvidia.com/isaac_ros/isaac_ros_benchmark/-/blob/dev/benchmarks/isaac_ros_nitros_bridge_benchmark/scripts/isaac_ros_nitros_bridge_reference.py).

## Documentation

Please visit the [Isaac ROS Documentation](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_nitros_bridge_ros1`](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros1/index.html)
  * [API](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros1/index.html#api)
* [`isaac_ros_nitros_bridge_ros2`](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros2/index.html)
  * [API](https://isaac_ros.gitlab-master-pages.nvidia.com/isaac_ros_docs/repositories_and_packages/isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros2/index.html#api)

## Latest

Update 2024-05-30: Update to be compatible with JetPack 6.0
