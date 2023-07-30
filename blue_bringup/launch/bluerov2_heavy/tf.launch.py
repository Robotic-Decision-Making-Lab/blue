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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the static TF broadcasters."""
    args = [
        DeclareLaunchArgument(
            "prefix", default_value="", description="The URDF model prefix."
        )
    ]

    prefix = LaunchConfiguration("prefix")

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster1",
            arguments=[
                "--x",
                "0.14",
                "--y",
                "-0.092",
                "--z",
                "0.0",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "-0.785",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster1"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster2",
            arguments=[
                "--x",
                "0.14",
                "--y",
                "0.092",
                "--z",
                "0.0",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "-2.356",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster2"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster3",
            arguments=[
                "--x",
                "-0.15",
                "--y",
                "-0.092",
                "--z",
                "0.0",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "0.785",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster3"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster4",
            arguments=[
                "--x",
                "-0.15",
                "--y",
                "0.092",
                "--z",
                "0.0",
                "--roll",
                "-1.571",
                "--pitch",
                "1.571",
                "--yaw",
                "2.356",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster4"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster5",
            arguments=[
                "--x",
                "0.118",
                "--y",
                "-0.215",
                "--z",
                "0.064",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster5"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster6",
            arguments=[
                "--x",
                "0.118",
                "--y",
                "0.215",
                "--z",
                "0.064",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster6"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster7",
            arguments=[
                "--x",
                "-0.118",
                "--y",
                "-0.215",
                "--z",
                "0.064",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster7"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_thruster8",
            arguments=[
                "--x",
                "-0.118",
                "--y",
                "0.215",
                "--z",
                "0.064",
                "--frame-id",
                [prefix, "base_link"],
                "--child-frame-id",
                [prefix, "thruster8"],
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link",
            arguments=[
                "--x",
                "-0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--frame-id",
                [prefix, "base_footprint"],
                "--child-frame-id",
                [prefix, "base_link"],
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(args + nodes)
