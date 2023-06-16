# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the system.

    Returns:
        The Blue ROS 2 launch description.
    """
    # Declare the launch arguments
    args = [
        DeclareLaunchArgument(
            "config",
            default_value="blue.yaml",
            description="The ROS 2 parameters configuration file.",
        ),
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
            default_value="mocap",
            choices=["mocap", "camera"],
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
            "description_package",
            default_value="blue_description",
            description=(
                "The description package with the BlueROV2 models. This is typically"
                " not set, but is available in case another description package has"
                " been defined"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Automatically start Gazebo.",
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="bluerov2_heavy_underwater.world",
            description="The world configuration to load if using Gazebo.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    use_sim = LaunchConfiguration("use_sim")
    gazebo_world_file = LaunchConfiguration("gazebo_world_file")

    config_filepath = PathJoinSubstitution(
        [
            FindPackageShare("blue_bringup"),
            "config",
            LaunchConfiguration("config"),
        ]
    )

    nodes = [
        Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            parameters=[config_filepath],
        ),
    ]

    processes = [
        ExecuteProcess(
            cmd=[
                "gz",
                "sim",
                "-v",
                "3",
                "-r",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "gazebo",
                        "worlds",
                        gazebo_world_file,
                    ]
                ),
            ],
            output="screen",
            condition=IfCondition(use_sim),
        ),
    ]

    # Declare additional launch files to run
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_manager"), "manager.launch.py"]
                )
            ),
            launch_arguments={"config_filepath": config_filepath}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_control"), "launch", "control.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": config_filepath,
                "controller": LaunchConfiguration("controller"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_localization"), "localization.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": config_filepath,
                "localization_source": LaunchConfiguration("localization_source"),
                "use_mocap": LaunchConfiguration("use_mocap"),
                "use_camera": LaunchConfiguration("use_camera"),
            }.items(),
            condition=UnlessCondition(use_sim),
        ),
    ]

    return LaunchDescription(args + nodes + processes + includes)
