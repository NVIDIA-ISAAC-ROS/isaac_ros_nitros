# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for DOPE encoder->TensorRT->DOPE decoder."""
    launch_args = [
        DeclareLaunchArgument(
            'output_image_width',
            default_value='640',
            description='The expected output image width from custom encoder'),
        DeclareLaunchArgument(
            'output_image_height',
            default_value='480',
            description='The expected output image height from custom encoder'),
        DeclareLaunchArgument(
            'input_image_width',
            default_value='1920',
            description='The expected output image width from custom encoder'),
        DeclareLaunchArgument(
            'input_image_height',
            default_value='1080',
            description='The expected output image height from custom encoder'),
        DeclareLaunchArgument(
            'image_mean',
            default_value='[0.5, 0.5, 0.5]',
            description='The mean for image normalization'),
        DeclareLaunchArgument(
            'image_stddev',
            default_value='[0.5, 0.5, 0.5]',
            description='The standard deviation for image normalization'),
        DeclareLaunchArgument(
            'model_file_path',
            description='The absolute file path to the ONNX file'),
        DeclareLaunchArgument(
            'engine_file_path',
            default_value='/tmp/trt_engine.plan',
            description='The absolute file path to the TensorRT engine file'),
        DeclareLaunchArgument(
            'input_tensor_names',
            default_value='["input_tensor"]',
            description='A list of tensor names to bound to the specified input binding names'),
        DeclareLaunchArgument(
            'input_binding_names',
            default_value='["input"]',
            description='A list of input tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'input_tensor_formats',
            default_value='["nitros_tensor_list_nchw_rgb_f32"]',
            description='The nitros format of the input tensors'),
        DeclareLaunchArgument(
            'output_tensor_names',
            default_value='["output"]',
            description='A list of tensor names to bound to the specified output binding names'),
        DeclareLaunchArgument(
            'output_binding_names',
            default_value='["output"]',
            description='A  list of output tensor binding names (specified by model)'),
        DeclareLaunchArgument(
            'output_tensor_formats',
            default_value='["nitros_tensor_list_nhwc_rgb_f32"]',
            description='The nitros format of the output tensors'),
        DeclareLaunchArgument(
            'tensorrt_verbose',
            default_value='False',
            description='Whether TensorRT should verbosely log or not'),
        DeclareLaunchArgument(
            'object_name',
            default_value='Ketchup',
            description='The object class that the DOPE network is detecting'),
        DeclareLaunchArgument(
            'force_engine_update',
            default_value='False',
            description='Whether TensorRT should update the TensorRT engine file or not'),
    ]

    # Custom Image Encoder parameters
    input_image_width = LaunchConfiguration('input_image_width')
    input_image_height = LaunchConfiguration('input_image_height')
    network_image_width = LaunchConfiguration('output_image_width')
    network_image_height = LaunchConfiguration('output_image_height')
    encoder_image_mean = LaunchConfiguration('image_mean')
    encoder_image_stddev = LaunchConfiguration('image_stddev')

    # Tensor RT parameters
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    input_tensor_names = LaunchConfiguration('input_tensor_names')
    input_binding_names = LaunchConfiguration('input_binding_names')
    input_tensor_formats = LaunchConfiguration('input_tensor_formats')
    output_tensor_names = LaunchConfiguration('output_tensor_names')
    output_binding_names = LaunchConfiguration('output_binding_names')
    output_tensor_formats = LaunchConfiguration('output_tensor_formats')
    tensorrt_verbose = LaunchConfiguration('tensorrt_verbose')
    force_engine_update = LaunchConfiguration('force_engine_update')

    # DOPE Decoder parameters
    object_name = LaunchConfiguration('object_name')

    encoder_node = ComposableNode(
        name='encoder_node',
        package='custom_nitros_dnn_image_encoder',
        plugin='custom_nitros_dnn_image_encoder::ImageEncoderNode',
        parameters=[{
            'output_image_width': network_image_width,
            'output_image_height': network_image_height,
            'image_mean': encoder_image_mean,
            'image_stddev': encoder_image_stddev,
            'input_image_height': input_image_height,
            'input_image_width': input_image_width
        }],
        remappings=[('/encoded_tensor', '/tensor_pub')]
    )

    dope_inference_node = ComposableNode(
        name='dope_inference',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'input_tensor_names': input_tensor_names,
            'input_binding_names': input_binding_names,
            'input_tensor_formats': input_tensor_formats,
            'output_tensor_names': output_tensor_names,
            'output_binding_names': output_binding_names,
            'output_tensor_formats': output_tensor_formats,
            'verbose': tensorrt_verbose,
            'force_engine_update': force_engine_update
        }])

    dope_decoder_node = ComposableNode(
        name='dope_decoder',
        package='isaac_ros_dope',
        plugin='nvidia::isaac_ros::dope::DopeDecoderNode',
        parameters=[{
            'object_name': object_name,
        }],
        remappings=[('belief_map_array', 'tensor_sub'),
                    ('dope/pose_array', 'poses')])

    container = ComposableNodeContainer(
        name='dope_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[encoder_node, dope_inference_node, dope_decoder_node],
        output='screen',
    )

    final_launch_description = launch_args + [container]
    return launch.LaunchDescription(final_launch_description)
