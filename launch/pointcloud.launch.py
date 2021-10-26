import os

import launch_ros
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.descriptions import ComposableNode

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find(
        "kinect_ros2"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            ComposableNodeContainer(
                name="kinect",
                namespace="kinect",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="kinect_ros2",
                        plugin="kinect_ros2::KinectRosComponent",
                        name="kinect_component",
                        namespace="kinect"
                    ),
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzNode",
                        name="point_cloud_xyz_node",
                        namespace="kinect",
                        remappings=[
                            ("image_rect", "depth/image_raw"),
                            ("camera_info", "depth/camera_info"),
                        ],
                    ),
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
        ]
    )
