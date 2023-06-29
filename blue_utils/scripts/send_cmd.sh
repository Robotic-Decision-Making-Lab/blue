#!/usr/bin/env bash

# Send a velocity command to the BlueROV2

ros2 topic pub /blue/ismc/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
