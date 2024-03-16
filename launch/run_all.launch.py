# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    ld = launch.LaunchDescription()
    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense2_camera',
        namespace='',
        parameters=[{
            'color_height': 1080,
            'color_width': 1920,
            'enable_infra1': True,
            'enable_infra2': True,
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
        remappings=[('/color/image_raw', '/image'),
                    ('/color/camera_info', '/camera_info')]
    )

    # AprilTag node

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
        }]
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace=''
    )
    apriltag_comms_node = Node(
        name='comms_node',
        namespace='',
        package='comms_node',
        executable='apriltag_subscriber',
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            realsense_camera_node
        ],
        output='screen'
    )

    # Detect node

    resize_node = ComposableNode(
        name='resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace='',
        parameters=[{
            'output_width': 640,
            'output_height': 640,
        }]
    )

    encoder_node = ComposableNode(
        name='dnn_image_encoder',
        package='isaac_ros_dnn_image_encoder',
        plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        remappings=[
            ('encoded_tensor', 'tensor_pub'),
            ('image', 'resize/image')
        ],
        parameters=[{
            'input_image_width': 640,
            'input_image_height': 640,
            'network_image_width': 640,
            'network_image_height': 640,
            'image_mean': [0.0, 0.0, 0.0],
            'image_stddev': [1.0, 1.0, 1.0],
        }],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': '/workspaces/isaac_ros-dev/src/jetson-localization/yolov8s.onnx',
            'engine_file_path': '/workspaces/isaac_ros-dev/src/jetson-localization/yolov8s.plan',
            'output_binding_names': ["output0"],
            'output_tensor_names': ["output_tensor"],
            'input_tensor_names': ["input_tensor"],
            'input_binding_names': ["images"],
            'verbose': False,
            'force_engine_update': False
        }]
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': 0.25,
            'nms_threshold': 0.45,
        }]
    )

    detect_comms_node = Node(
        name='comms_node',
        namespace='',
        package='comms_node',
        executable='detect_subscriber',
    )

    tensor_rt_container = ComposableNodeContainer(
        name='tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            encoder_node,
            tensor_rt_node,
            yolov8_decoder_node,
            realsense_camera_node,
            resize_node
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace=''
    )

    # Visual SLAM node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'camera_link',
            'input_imu_frame': 'camera_gyro_optical_frame',
            'enable_imu_fusion': False,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'img_jitter_threshold_ms': 22.00
        }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    vslam_comms_node = Node(
        name='comms_node',
        namespace='',
        package='comms_node',
        executable='vslam_subscriber',
    )
    ld.add_action(realsense_camera_node)
    ld.add_action(apriltag_container)
    ld.add_action(apriltag_comms_node)
    ld.add_action(detect_comms_node)
    ld.add_action(tensor_rt_container)
    ld.add_action(visual_slam_launch_container)
    ld.add_action(vslam_comms_node)
    return ld
