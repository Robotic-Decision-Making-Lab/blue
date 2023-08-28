# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the system.

    Returns:
        The launch description for the BlueROV2 base configuration.
    """
    args = [
        DeclareLaunchArgument(
            "controller",
            default_value="ismc",
            description=(
                "The controller to use; this should be the same name as the"
                " controller's executable."
            ),
            choices=["ismc"],
        ),
        DeclareLaunchArgument(
            "localization_source",
            default_value="gazebo",
            choices=["mocap", "camera", "gazebo", "hinsdale", "dvl"],
            description="The localization source to stream from.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description=(
                "Launch the BlueROV2 camera stream. This is automatically set to true"
                " when using the camera for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_mocap",
            default_value="false",
            description=(
                "Launch the Qualisys motion capture stream. This is automatically"
                " set to true when using the motion capture system for localization."
            ),
        ),
        DeclareLaunchArgument(
            "use_dvl",
            default_value="false",
            description="Launch the DVL interface.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", description="Launch RViz2."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="bluerov2_heavy_reach.rviz",
            description="The RViz2 configuration file to load.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the model. This is useful for multi-robot setups."
                " Expected format '<prefix>/'."
            ),
        ),
    ]

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("blue_description"),
                    "xacro",
                    "bluerov2_heavy_reach",
                    "config.xacro",
                ]
            ),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "use_sim:=",
            LaunchConfiguration("use_sim"),
        ]
    )

    return LaunchDescription(
        [
            *args,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("blue_bringup"), "launch", "base.launch.py"]
                    )
                ),
                launch_arguments={
                    "configuration_type": "bluerov2_heavy_reach",
                    "controller": LaunchConfiguration("controller"),
                    "localization_source": LaunchConfiguration("localization_source"),
                    "use_camera": LaunchConfiguration("use_camera"),
                    "use_mocap": LaunchConfiguration("use_mocap"),
                    "use_dvl": LaunchConfiguration("use_dvl"),
                    "use_sim": LaunchConfiguration("use_sim"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "gazebo_world_file": "bluerov2_heavy_reach_underwater.world",  # noqa
                    "prefix": LaunchConfiguration("prefix"),
                    "robot_description": robot_description,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("blue_bringup"),
                            "launch",
                            "bluerov2_heavy_reach",
                            "tf.launch.py",
                        ]
                    )
                ),
                launch_arguments={"prefix": LaunchConfiguration("prefix")}.items(),
            ),
        ]
    )
