# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'enable_depth': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.profile': '640x360x90',
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 200,
            'unite_imu_method': 2
        }],
        remappings=[('/camera/color/image_raw', '/image')]
    )

    encoder_node = ComposableNode(
            package='isaac_ros_dnn_image_encoder',
            plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
            name='dnn_image_encoder',
            parameters=[{
                'input_image_width': 640,
                'input_image_height': 640,
                'network_image_width': 640,
                'network_image_height': 640,
                'image_mean': [0.0, 0.0, 0.0],
                'image_stddev': [1.0, 1.0, 1.0],
            }]
        )

    tensor_rt_node = ComposableNode(
            package='isaac_ros_tensor_rt',
            plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
            name='tensor_rt',
            parameters=[{
                'model_file_path': '/workspaces/isaac_ros-dev/src/jetson-localization/yolov8s.onnx',
                'engine_file_path': '/tmp/yolov8s.plan',
                'input_tensor_names': ['images'],
                'output_tensor_names': ['output0'],
                'input_binding_names': ['images'],
                'output_binding_names': ['output0'],
                'verbose': False,
                'force_engine_update': False
            }]
        )

    yolov8_decoder_node = ComposableNode(
            package='isaac_ros_yolov8',
            plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
            name='yolov8_decoder_node',
            parameters=[{
                'confidence_threshold': 0.25,
                'nms_threshold': 0.45,
            }]
        )

    tensor_rt_container = ComposableNodeContainer(
        name='tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[encoder_node, tensor_rt_node, yolov8_decoder_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace=''
    )

    comms_node = Node(
        name='comms_node',
        namespace='',
        package='comms_node',
        executable='detect_subscriber',
    )

    return launch.LaunchDescription([tensor_rt_container, realsense_camera_node, comms_node])
