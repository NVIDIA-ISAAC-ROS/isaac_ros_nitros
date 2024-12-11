# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from glob import glob
from os import path

from setuptools import find_packages, setup

package_name = 'isaac_ros_pynitros'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=[
        'cuda_python',
        'posix_ipc',
        'setuptools'
    ],
    zip_safe=True,
    maintainer='Isaac ROS Maintainers',
    maintainer_email='isaac-ros-maintainers@nvidia.com',
    description='Execute the control graph in isaac ros integration test',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pynitros_image_forward_node = \
                isaac_ros_pynitros.examples.pynitros_image_forward_node:main',
            'pynitros_tensor_list_forward_node = \
                isaac_ros_pynitros.examples.pynitros_tensor_list_forward_node:main',
            'pynitros_dnn_image_encoder_node = \
                isaac_ros_pynitros.examples.pynitros_dnn_image_encoder_node:main',
            'pynitros_message_filter_sync_node = \
                isaac_ros_pynitros.examples.pynitros_message_filter_sync_node:main',
        ],
    },
)
