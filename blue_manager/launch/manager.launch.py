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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the Blue manager interface.

    Returns:
        LaunchDescription: The Blue manager launch description.
    """
    args = [
        DeclareLaunchArgument(
            "manager_filepath",
            default_value=None,
            description="The path to the manager configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "joy_filepath",
            default_value=None,
            description="The path to the joystick controller configuration YAML file.",
        ),
        DeclareLaunchArgument(
            "use_joy", default_value="false", description="Use a joystick controller."
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="The full path to the joystick device to use.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=("Use the simulated Gazebo clock."),
        ),
    ]

    nodes = [
        Node(
            package="blue_manager",
            executable="blue_manager",
            name="blue_manager",
            output="both",
            parameters=[
                LaunchConfiguration("manager_filepath"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="blue_manager",
            executable="joy_controller",
            name="joy_controller",
            output="both",
            parameters=[
                LaunchConfiguration("joy_filepath"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            condition=IfCondition(LaunchConfiguration("use_joy")),
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            output="both",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "dev": LaunchConfiguration("joy_device"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_joy")),
        ),
    ]

    return LaunchDescription(args + nodes)
