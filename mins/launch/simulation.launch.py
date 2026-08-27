import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    default_config = PathJoinSubstitution(
        [FindPackageShare("mins"), "config", "simulation", "config.yaml"]
    )
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value=default_config,
            description="Path to the master MINS simulation config yaml",
        ),
        Node(
            package="mins",
            executable="simulation",
            name="mins_simulation",
            output="screen",
            parameters=[{"config_path": config_path}],
        ),
    ])
