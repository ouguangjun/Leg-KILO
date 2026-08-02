from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("legkilo")
    config_file = LaunchConfiguration("config_file")
    legkilo_robot_rviz = PathJoinSubstitution([package_share, "rviz", "legkilo_robot.rviz"])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution([package_share, "config", "legkilo_go1_velodyne.yaml"]),
            ),
            Node(
                package="legkilo",
                executable="legkilo_node",
                name="legkilo_node",
                output="screen",
                arguments=["--config_file", config_file],
            ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz",
            #     output="screen",
            #     arguments=["-d", legkilo_robot_rviz],
            #     prefix="nice",
            # )
            #
            # Node(
            #     package="robot_state_publisher",
            #     executable="robot_state_publisher",
            #     name="robot_state_publisher",
            #     parameters=[
            #         {
            #             "publish_frequency": 1000.0,
            #             "robot_description": Command(["cat ", go1_urdf]),
            #         }
            #     ],
            # )
            #
            # launch-prefix="gdb -ex run --args"
        ]
    )
