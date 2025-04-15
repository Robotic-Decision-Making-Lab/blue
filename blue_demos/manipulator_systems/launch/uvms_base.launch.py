# Copyright 2025, Evan Palmer
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("gz_model_name"),
        DeclareLaunchArgument("gz_world_file"),
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_description"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
                "use_sim_time": "true",
            }
        ],
    )

    # Gazebo launch
    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("gz_model_name"),
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ),
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    # we need this physics engine for mimic joints
                    "-v 4 --physics-engine gz-physics-bullet-featherstone-plugin -r",
                    " ",
                    LaunchConfiguration("gz_world_file"),
                ],
            )
        ],
    )

    # the velocity controller expects state information to be provided in the FSD frame
    # message_transformer = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("message_transforms"),
    #                 "launch",
    #                 "message_transforms.launch.py",
    #             ]
    #         )
    #     ),
    #     launch_arguments={
    #         "parameters_file": PathJoinSubstitution(
    #             [
    #                 FindPackageShare("blue_demos"),
    #                 "manipulator_systems",
    #                 "config",
    #                 "uvms_transforms.yaml",
    #             ]
    #         ),
    #         "ns": TextSubstitution(text="manipulator_systems"),
    #     }.items(),
    # )

    # extend the controller launch timeouts
    # gazebo can take a while to get started, so we need some extra time for that
    controller_timeout = ["--controller-manager-timeout", "120"]
    switch_timeout = ["--switch-timeout", "100"]

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "adaptive_integral_terminal_sliding_mode_controller",
            "--controller-manager",
            ["", "controller_manager"],
            *controller_timeout,
            *switch_timeout,
        ],
    )

    tam_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thruster_allocation_matrix_controller",
            "--controller-manager",
            ["", "controller_manager"],
            *controller_timeout,
            *switch_timeout,
        ],
    )

    thruster_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                f"thruster{i + 1}_controller",
                "--controller-manager",
                ["", "controller_manager"],
                *controller_timeout,
                *switch_timeout,
            ],
        )
        for i in range(8)
    ]

    # launch the thrusters sequentially
    delay_thruster_controller_spawners = []
    for i, thruster_spawner in enumerate(thruster_spawners):
        if not len(delay_thruster_controller_spawners):
            delay_thruster_controller_spawners.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=gz_spawner, on_exit=[thruster_spawner]
                    )
                )
            )
        else:
            delay_thruster_controller_spawners.append(
                RegisterEventHandler(
                    event_handler=OnExecutionComplete(
                        target_action=thruster_spawners[i - 1],
                        on_completion=[thruster_spawner],
                    )
                )
            )

    # launch the TAM controller after the thruster controllers
    delay_tam_controller_spawner_after_thruster_controller_spawners = (
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=thruster_spawners[-1],
                on_completion=[tam_controller_spawner],
            )
        )
    )

    # launch the velocity controller after the TAM controller
    delay_velocity_controller_spawner_after_tam_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=tam_controller_spawner,
                on_completion=[velocity_controller_spawner],
            )
        )
    )

    return LaunchDescription(
        [
            *args,
            # message_transformer,
            robot_state_publisher,
            gz_spawner,
            gz_launch,
            *delay_thruster_controller_spawners,
            delay_tam_controller_spawner_after_thruster_controller_spawners,
            delay_velocity_controller_spawner_after_tam_controller_spawner,
        ]
    )
