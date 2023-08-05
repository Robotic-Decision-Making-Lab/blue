# Blue :ocean:

Blue is an end-to-end ROS 2 pipeline designed to support development,
testing, and deployment of underwater robots.

## Main Features

When you use Blue, you get access to the following set of features:

- A suite of interfaces designed to enable development of custom localization,
  planning, and control algorithms alongside the existing implemented algorithms
  (e.g., integral sliding mode control)
- A collection of commonly-used underwater vehicle models (both visual and
  hydrodynamic) like the [Blue Robotics](https://bluerobotics.com/) BlueROV2
  and the BlueROV2 Heavy with support for developing your own custom models
- ArduSub + Gazebo SITL integration for evaluating the performance of your
  algorithms in a simulation environment
- Docker integration to support deployment of your algorithms to hardware

## Installation

Blue is currently supported on Linux, and is available for the ROS
distributions Humble, Iron, and Rolling. To install Blue, first clone this
project to the `src` directory of your ROS workspace, replacing `$ROS_DISTRO`
with the desired ROS distribution or `main` for Rolling:

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

ROS 2 launch files have been provided to start Blue. To launch the simulation
environment for the BlueROV2 Heavy, run

```bash
ros2 launch blue_bringup bluerov2_heavy.launch.py use_sim:=true
```

A full description of the launch arguments and their respective default values
can be obtained by running the following command:

```bash
ros2 launch blue_bringup bluerov2_heavy.launch.py --show-args
```

## Getting help

If you have questions regarding usage of Blue or regarding contributing to this
project, please ask a question on our [Discussions](https://github.com/evan-palmer/blue/discussions)
board!

## License

The Blue is released under the MIT license.
