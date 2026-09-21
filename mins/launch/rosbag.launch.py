import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    path_bag = LaunchConfiguration("path_bag")
    config_path = PathJoinSubstitution(
        [FindPackageShare("mins"), "config", config, "config.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mins"), "launch", "display.rviz"]
    )
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="udesa",
            description="Config folder under mins/config (e.g. udesa, kaist/kaist_LC)",
        ),
        DeclareLaunchArgument(
            "path_bag",
            description="rosbag2 bag: a bag directory, a single .mcap/.db3 file, "
                        "or several of them separated with ':' (merged by time)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Also start RViz with the MINS display config",
        ),
        Node(
            package="mins",
            executable="bag",
            name="mins_bag",
            output="screen",
            arguments=[config_path],
            parameters=[{"config_path": config_path, "sys_path_bag": path_bag}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
