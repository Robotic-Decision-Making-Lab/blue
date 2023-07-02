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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the system.

    Returns:
        The launch description for the BlueROV2 configuration.
    """
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="blue_description",
            description=(
                "The description package with the blue configuration files. This is"
                " typically not set, but is available in case another description"
                " package has been defined."
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="bluerov2_controllers.yaml",
            description="The BlueROV2 controller configuration file.",
        ),
        DeclareLaunchArgument(
            "localization_file",
            default_value="bluerov2_controllers.yaml",
            description="The BlueROV2 controller configuration file.",
        ),
        DeclareLaunchArgument(
            "manager_file",
            default_value="bluerov2_manager.yaml",
            description="The BlueROV2 manager configuration file.",
        ),
        DeclareLaunchArgument(
            "mavros_file",
            default_value="mavros.yaml",
            description="The MAVROS configuration file.",
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
            default_value="gazebo",
            choices=["mocap", "camera", "gazebo"],
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
            "use_sim",
            default_value="false",
            description="Launch the Gazebo + ArduSub simulator.",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="false",
            description="Start the Foxglove bridge.",
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="bluerov2_underwater.world",
            description="The world configuration to load if using Gazebo.",
        ),
        DeclareLaunchArgument(
            "ardusub_params_file",
            default_value="bluerov2.parm",
            description=(
                "The ArduSub parameters that the BlueROV2 should use if running in"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "foxglove_bridge_address",
            default_value="127.0.0.1",
            description="The Foxglove Studio datasource address.",
        ),
        DeclareLaunchArgument(
            "foxglove_bridge_port",
            default_value="8765",
            description="The Foxglove Studio datasource port.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    controllers_file = LaunchConfiguration("controllers_file")
    localization_file = LaunchConfiguration("localization_file")
    manager_file = LaunchConfiguration("manager_file")
    mavros_file = LaunchConfiguration("mavros_file")
    use_sim = LaunchConfiguration("use_sim")

    ardusub_params_filepath = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "ardusub",
            LaunchConfiguration("ardusub_params_file"),
        ]
    )

    nodes = [
        Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", mavros_file]
                )
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            output="screen",
        ),
    ]

    processes = [
        # Launch Ignition Gazebo - this launches Garden (what ardupilot_gazebo uses)
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
                        LaunchConfiguration("gazebo_world_file"),
                    ]
                ),
            ],
            output="screen",
            condition=IfCondition(use_sim),
        ),
        # Launch ArduSub for simulation purposes
        ExecuteProcess(
            cmd=[
                "ardusub",
                "-S",
                "-w",
                "-M",
                "JSON",
                "--defaults",
                ardusub_params_filepath,
                "-I0",
                "--home",
                "44.65870,-124.06556,0.0,270.0",  # my not-so-secrect surf spot
            ],
            output="screen",
            condition=IfCondition(use_sim),
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_manager"), "manager.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", manager_file]
                )
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_control"), "launch", "control.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", controllers_file]
                ),
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
                "config_filepath": PathJoinSubstitution(
                    [FindPackageShare(description_package), "config", localization_file]
                ),
                "localization_source": LaunchConfiguration("localization_source"),
                "use_mocap": LaunchConfiguration("use_mocap"),
                "use_camera": LaunchConfiguration("use_camera"),
            }.items(),
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("foxglove_bridge"),
                        "foxglove_bridge_launch.xml",
                    ]
                ),
            ),
            launch_arguments={
                "address": LaunchConfiguration("foxglove_bridge_address"),
                "port": LaunchConfiguration("foxglove_bridge_port"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_foxglove")),
        ),
    ]

    return LaunchDescription(args + nodes + processes + includes)
