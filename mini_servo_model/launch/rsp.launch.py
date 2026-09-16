from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "urdf", "mini_servo.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                urdf_path,
                " ",
                "ros2_control_hardware_type:=",
                ros2_control_hardware_type,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="mock_components",
                description="ros2_control hardware plugin type (mock_components or gz_ros2_control)",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
