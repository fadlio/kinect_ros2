import launch_ros

import launch


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="kinect_ros2",
                executable="kinect_ros2",
                name="kinect_ros2",
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="rgb_showimage",
                parameters=[{"window_name": "RGB"}],
                remappings=[("image", "image_raw")],
            ),
            launch_ros.actions.Node(
                package="image_tools",
                executable="showimage",
                name="depth_showimage",
                parameters=[{"window_name": "Depth"}],
                remappings=[("image", "depth/image_raw")],
            ),
        ]
    )
