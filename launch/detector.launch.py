# Copyright 2024 YOLOs-CPP Team
# SPDX-License-Identifier: AGPL-3.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch YOLO detector node as composable lifecycle node."""

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        "model_path", description="Path to ONNX model file"
    )
    labels_path_arg = DeclareLaunchArgument(
        "labels_path", description="Path to class names file"
    )
    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu", default_value="false", description="Enable GPU inference"
    )
    conf_threshold_arg = DeclareLaunchArgument(
        "conf_threshold", default_value="0.8", description="Confidence threshold"
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/camera/image_raw", description="Input image topic"
    )
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Whether to start usb_cam together with detector",
    )
    camera_params_file_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ros2_yolos_cpp"), "config", "usb_cam_params.yaml"]
        ),
        description="Path to usb_cam parameter yaml",
    )
    service_name_arg = DeclareLaunchArgument(
        "service_name",
        default_value="~/detect",
        description="Detection service name for request-driven inference",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ros2_yolos_cpp"), "config", "default_params.yaml"]
        ),
        description="Path to detector parameter yaml",
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        parameters=[LaunchConfiguration("camera_params_file")],
        remappings=[
            ("/image_raw", LaunchConfiguration("image_topic")),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # Composable node container (multi-threaded for async inference)
    container = ComposableNodeContainer(
        name="yolos_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_yolos_cpp",
                plugin="ros2_yolos_cpp::YolosDetectorNode",
                name="yolos_detector",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "labels_path": LaunchConfiguration("labels_path"),
                        "use_gpu": LaunchConfiguration("use_gpu"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "nms_threshold": 0.45,
                        "publish_timing": True,
                    }
                ],
                remappings=[
                    ("~/image_raw", LaunchConfiguration("image_topic")),
                    ("~/detect", LaunchConfiguration("service_name")),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_path_arg,
            labels_path_arg,
            use_gpu_arg,
            conf_threshold_arg,
            image_topic_arg,
            start_camera_arg,
            camera_params_file_arg,
            service_name_arg,
            params_file_arg,
            camera_node,
            container,
        ]
    )
