from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "mini_servo", package_name="mini_servo_moveit"
    ).to_moveit_configs()

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ros2_controllers = str(moveit_config.package_path / "config/ros2_controllers.yaml")

    nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--param-file", ros2_controllers],
            output="screen",
        )
        for controller in controller_names
    ]
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        )
    )
    return LaunchDescription(nodes)
