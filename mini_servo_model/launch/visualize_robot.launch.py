from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "urdf", "mini_servo.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "rviz", "visualize_robot.rviz"]
    )
    shutdown_wrapper = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "scripts", "shutdown_wrapper.py"]
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

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        prefix=shutdown_wrapper,
        sigterm_timeout="3",
        condition=IfCondition(use_gui),
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_gui),
    )
    joint_state_publisher_fallback = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    def fallback_joint_states(event: ProcessExited, _context):
        # Negative codes are signals (Ctrl-C). Only replace a real GUI crash.
        if event.returncode is None or event.returncode <= 0:
            return []
        return [joint_state_publisher_fallback]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start joint_state_publisher_gui to drag joints",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the visualization config",
            ),
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
            joint_state_publisher_gui,
            joint_state_publisher,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_publisher_gui,
                    on_exit=fallback_joint_states,
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                prefix=shutdown_wrapper,
                sigterm_timeout="3",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
