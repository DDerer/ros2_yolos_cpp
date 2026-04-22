# Copyright 2024 YOLOs-CPP Team
# SPDX-License-Identifier: AGPL-3.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch YOLO pose node as composable lifecycle node."""
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        description='Path to ONNX model file'
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='false',
        description='Enable GPU inference'
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.4',
        description='Confidence threshold'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    container = ComposableNodeContainer(
        name='yolos_pose_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_yolos_cpp',
                plugin='ros2_yolos_cpp::YolosPoseNode',
                name='yolos_pose',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    # Pose models typically don't strictly need external labels for basic drawing,
                    # but config struct might expect it. However, PoseNode loadConfig implementation
                    # usually just needs model_path.
                    'use_gpu': LaunchConfiguration('use_gpu'),
                    'conf_threshold': LaunchConfiguration('conf_threshold'),
                    'nms_threshold': 0.45,
                    'publish_debug_image': True,
                }],
                remappings=[
                    ('~/image_raw', LaunchConfiguration('image_topic')),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        use_gpu_arg,
        conf_threshold_arg,
        image_topic_arg,
        container,
    ])
