from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dev", default_value="/dev/input/js0"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                parameters=[
                    {
                        "device_name": LaunchConfiguration("dev"),
                        "deadzone": 0.2,
                        "autorepeat_rate": 40.0,
                    }
                ],
            ),
            Node(
                package="moveit_ros_visualization",
                executable="moveit_joy.py",
                name="moveit_joy",
                output="screen",
            ),
        ]
    )
