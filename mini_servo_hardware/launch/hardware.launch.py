from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_rviz = LaunchConfiguration("use_rviz")

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("mini_servo_hardware"), "config", "ros2_controllers.yaml"]
    )
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "urdf", "mini_servo.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mini_servo_model"), "rviz", "visualize_robot.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    urdf_xacro,
                    " ",
                    "ros2_control_hardware_type:=mini_servo_hardware",
                    " ",
                    "serial_port:=",
                    serial_port,
                    " ",
                    "baud_rate:=",
                    baud_rate,
                ]
            ),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", controllers_yaml],
    )
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", controllers_yaml],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device for the Arduino",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="500000",
                description="Serial baud rate (must match mini_servo.ino)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the arm visualization",
            ),
            robot_state_publisher,
            ros2_control_node,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control_node,
                    on_start=[
                        joint_state_broadcaster_spawner,
                        forward_position_controller_spawner,
                    ],
                )
            ),
            rviz,
        ]
    )
