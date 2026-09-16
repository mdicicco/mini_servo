import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    model_share = get_package_share_directory("mini_servo_model")
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("mini_servo_gazebo"), "config", "ros2_controllers.yaml"]
    )
    bridge_yaml = PathJoinSubstitution(
        [FindPackageShare("mini_servo_gazebo"), "config", "ros_gz_bridge.yaml"]
    )
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "urdf", "mini_servo.urdf.xacro"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    urdf_xacro,
                    " ",
                    "ros2_control_hardware_type:=gz_ros2_control",
                    " ",
                    "use_gazebo:=true",
                    " ",
                    "gazebo_controllers_file:=",
                    controllers_yaml,
                ]
            ),
            value_type=str,
        )
    }

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={
            "gz_args": "-r -v 1 empty.sdf",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gui),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={
            "gz_args": "--headless-rendering -s -r -v 1 empty.sdf",
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "mini_servo",
            "-allow_renaming",
            "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controllers_yaml,
        ],
    )
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--param-file",
            controllers_yaml,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock as ROS time",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo GUI (false runs headless)",
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.dirname(model_share),
            ),
            gz_sim_gui,
            gz_sim_headless,
            gz_bridge,
            robot_state_publisher,
            gz_spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[forward_position_controller_spawner],
                )
            ),
        ]
    )
