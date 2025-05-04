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
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("gz_model_name"),
        DeclareLaunchArgument("gz_world_file"),
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_description"),
        DeclareLaunchArgument("use_trajectory_control"),
        DeclareLaunchArgument("use_target"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
                "use_sim_time": True,
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
                ["-v 4 -r", " ", LaunchConfiguration("gz_world_file")],
            )
        ],
    )

    # the velocity controller expects state information to be provided in the FSD frame
    message_transformer = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("message_transforms"),
                "launch",
                "message_transforms.launch.yaml",
            )
        ),
        launch_arguments={
            "parameters_file": os.path.join(
                get_package_share_directory("blue_demos"),
                "manipulator_systems",
                "config",
                "uvms_transforms.yaml",
            ),
            "ns": TextSubstitution(text="manipulator_systems"),
        }.items(),
    )

    # spawn a target pose for the whole body controller
    spawn_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-file",
            os.path.join(
                get_package_share_directory("blue_demos"),
                "manipulator_systems",
                "description",
                "sdf",
                "target.sdf",
            ),
            "-name",
            "target",
            "-x",
            "1",
            "-y",
            "0",
            "-z",
            "-1",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_target")),
    )

    delay_target_spawner_after_gz_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=gz_spawner, on_exit=[spawn_pose])
    )

    def make_controller_args(name):
        cm = ["--controller-manager", ["", "controller_manager"]]
        controller_timeout = ["--controller-manager-timeout", "120"]
        switch_timeout = ["--switch-timeout", "100"]
        return [name, *cm, *controller_timeout, *switch_timeout]

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args(
            "adaptive_integral_terminal_sliding_mode_controller"
        ),
    )

    tam_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args("thruster_allocation_matrix_controller"),
    )

    thruster_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=make_controller_args(f"thruster{i + 1}_controller"),
        )
        for i in range(8)
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args("joint_state_broadcaster"),
    )

    tcp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args("tcp_position_controller"),
    )

    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args("whole_body_controller"),
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=make_controller_args("end_effector_trajectory_controller"),
        condition=IfCondition(LaunchConfiguration("use_trajectory_control")),
    )

    # launch the thrusters sequentially
    delay_thruster_controller_spawners = []
    for i, thruster_spawner in enumerate(thruster_controller_spawners):
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
                        target_action=thruster_controller_spawners[i - 1],
                        on_completion=[thruster_spawner],
                    )
                )
            )

    # launch the TAM controller after the thruster controllers
    delay_tam_controller_spawner_after_thruster_controller_spawners = (
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=thruster_controller_spawners[-1],
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

    delay_jsb_spawner_after_gz_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawner, on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_tcp_controller_spawner_after_jsb_spawner = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=joint_state_broadcaster_spawner,
            on_completion=[tcp_controller_spawner],
        )
    )

    delay_wbc_spawner_after_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=velocity_controller_spawner,
            on_completion=[whole_body_controller_spawner],
        )
    )

    delay_trajectory_controller_spawner_after_wbc_spawner = RegisterEventHandler(
        event_handler=OnExecutionComplete(
            target_action=whole_body_controller_spawner,
            on_completion=[trajectory_controller_spawner],
        ),
    )

    return LaunchDescription(
        [
            *args,
            message_transformer,
            robot_state_publisher,
            gz_spawner,
            gz_launch,
            delay_target_spawner_after_gz_spawner,
            *delay_thruster_controller_spawners,
            delay_tam_controller_spawner_after_thruster_controller_spawners,
            delay_velocity_controller_spawner_after_tam_controller_spawner,
            delay_jsb_spawner_after_gz_spawner,
            delay_tcp_controller_spawner_after_jsb_spawner,
            delay_wbc_spawner_after_velocity_controller_spawner,
            delay_trajectory_controller_spawner_after_wbc_spawner,
        ]
    )
