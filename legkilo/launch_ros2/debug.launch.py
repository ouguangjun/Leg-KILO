from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("legkilo")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution([package_share, "config", "leg_fusion.yaml"]),
            ),
            Node(
                package="legkilo",
                executable="legkilo_node",
                name="legkilo_node",
                output="screen",
                arguments=["--config_file", config_file],
                prefix="gdb -ex run --args",
            ),
            # launch-prefix="gdb -ex run --args"
        ]
    )
