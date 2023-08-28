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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to run the system.

    Returns:
        The base launch file for BlueROV2 configurations.
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
            "configuration_type",
            default_value="bluerov2_heavy",
            description="The BlueROV2 configuration type to load.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="The BlueROV2 Heavy controller configuration file.",
        ),
        DeclareLaunchArgument(
            "localization_file",
            default_value="localization.yaml",
            description="The BlueROV2 Heavy localization configuration file.",
        ),
        DeclareLaunchArgument(
            "manager_file",
            default_value="manager.yaml",
            description="The BlueROV2 Heavy manager configuration file.",
        ),
        DeclareLaunchArgument(
            "mavros_file",
            default_value="mavros.yaml",
            description="The MAVROS configuration file.",
        ),
        DeclareLaunchArgument(
            "ardusub_params_file",
            default_value="ardusub.parm",
            description=(
                "The ArduSub parameters that the BlueROV2 should use if running in"
                " simulation."
            ),
        ),
        DeclareLaunchArgument(
            "joy_file",
            default_value="joy_teleop.yaml",
            description="The joystick controller configuration file.",
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="",
            description="The world configuration to load if using Gazebo.",
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
            "use_rviz",
            default_value="false",
            description="Launch RViz2.",
        ),
        DeclareLaunchArgument(
            "use_joy", default_value="false", description="Use a joystick controller."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
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
        DeclareLaunchArgument(
            "robot_description",
            default_value="",
            description="The model URDF file.",
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="The full path to the joystick device to use.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    configuration_type = LaunchConfiguration("configuration_type")
    use_sim = LaunchConfiguration("use_sim")
    robot_description = LaunchConfiguration("robot_description")

    nodes = [
        Node(
            package="mavros",
            executable="mavros_node",
            output="both",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("mavros_file"),
                    ]
                ),
                {"use_sim_time": use_sim},
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {"use_sim_time": use_sim, "robot_description": robot_description}
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="both",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "rviz",
                        LaunchConfiguration("rviz_config"),
                    ]
                ),
            ],
            parameters=[
                {"use_sim_time": use_sim, "robot_description": robot_description}
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("blue_bringup"),
                            "launch",
                            "sitl.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "description_package": description_package,
                "configuration_type": configuration_type,
                "ardusub_params_file": LaunchConfiguration("ardusub_params_file"),
                "gazebo_world_file": LaunchConfiguration("gazebo_world_file"),
            }.items(),
            condition=IfCondition(use_sim),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("blue_manager"), "manager.launch.py"]
                )
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        configuration_type,
                        LaunchConfiguration("manager_file"),
                    ]
                ),
                "use_sim_time": use_sim,
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
                    [
                        FindPackageShare(description_package),
                        "config",
                        configuration_type,
                        LaunchConfiguration("controllers_file"),
                    ]
                ),
                "controller": LaunchConfiguration("controller"),
                "use_sim_time": use_sim,
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
                    [
                        FindPackageShare(description_package),
                        "config",
                        configuration_type,
                        LaunchConfiguration("localization_file"),
                    ]
                ),
                "localization_source": LaunchConfiguration("localization_source"),
                "use_mocap": LaunchConfiguration("use_mocap"),
                "use_camera": LaunchConfiguration("use_camera"),
                "use_sim_time": use_sim,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("blue_joy"), "joy.launch.py"])
            ),
            launch_arguments={
                "config_filepath": PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "config",
                        LaunchConfiguration("joy_file"),
                    ]
                ),
                "controller": LaunchConfiguration("controller"),
                "use_sim_time": use_sim,
            }.items(),
        ),
    ]

    return LaunchDescription(args + nodes + includes)
