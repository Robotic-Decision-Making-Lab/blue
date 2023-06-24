#!/usr/bin/env bash

# This is a simple script used to make your life easier when disarming a custom controller

ros2 service call /blue/control/arm std_srvs/srv/SetBool data:\ false\

ros2 service call /blue/manager/enable_passthrough std_srvs/srv/SetBool data:\ false\
