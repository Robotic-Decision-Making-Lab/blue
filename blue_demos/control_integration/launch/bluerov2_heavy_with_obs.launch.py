from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
    ]

    yaml_launch = IncludeLaunchDescription(
        YAMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("blue_demos"),
                    "control_integration",
                    "launch",
                    "bluerov2_heavy_demo_obs.launch.yaml",
                ]
            ),
        )
    )

    return LaunchDescription([*args, yaml_launch])
