# BlueROV2 ROS 2 driver :ocean:

The BlueROV2 driver is a collection of ROS 2 packages designed
to support ROS 2 integration with the [BlueROV2](https://bluerobotics.com/)
(both base and heavy configurations) and [ArduSub](https://www.ardusub.com/).

## Main Features

The main features of the project include:

- Support for developing custom controllers for the BlueROV2
- RC Passthrough mode: a custom mode which enables sending thrust commands to individual thrusters
- External localization support to enable developing custom SLAM algorithms and interfaces for localization sensors
- ArduSub + Gazebo SITL integration for evaluating the performance of your algorithms in a simulation environment
- Interfaces for adjusting hydrodynamic parameters

## Installation

The BlueROV2 driver is currently supported on Linux, and is available for the
ROS distributions Humble, Iron, and Rolling. To install the BlueROV2 driver,
first clone this project to the `src` directory of your ROS workspace, replacing
`$ROS_DISTRO` with the desired ROS distribution or `main` for Rolling:

```bash
git clone -b $ROS_DISTRO git@github.com:evan-palmer/blue.git
```

After cloning the project, install all external dependencies using `vcs`:

```bash
vcs import src < src/blue/blue.repos
```

Finally, install the ROS dependencies using `rosdep`, again, replacing
`$ROS_DISTRO` with the desired ROS distribution:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## Quick start

ROS 2 launch files have been provided to start the BlueROV2 driver for the base
and heavy configurations. To launch the system for the BlueROV2 Heavy, run

```bash
ros2 launch blue_bringup bluerov2_heavy.launch.py
```

A full description of the launch arguments and their respective default values
can be obtained by running the following command:

```bash
ros2 launch blue_bringup bluerov2_heavy.launch.py --show-args
```

## Getting help

If you have questions regarding usage of the BlueROV2 driver or regarding
contributing to this project, please ask a question on our
[Discussions](https://github.com/evan-palmer/blue/discussions) board!

## License

The BlueROV2 ROS 2 driver is released under the MIT license.
