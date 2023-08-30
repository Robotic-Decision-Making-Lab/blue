#!/usr/bin/env bash

# Start the keyboard teleop node for the ISMC controller

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/blue/ismc/cmd_vel
