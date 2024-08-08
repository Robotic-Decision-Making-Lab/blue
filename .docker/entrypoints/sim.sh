#!/usr/bin/env bash

# Add results of ArduSub build
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Add ardupilot_gazebo models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add blue models and worlds
export GZ_SIM_RESOURCE_PATH=$HOME/ws_blue/src/blue/blue_description/gazebo/models:$HOME/ws_blue/src/blue/blue_description/gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add blue meshes
export GZ_SIM_RESOURCE_PATH=$HOME/ws_blue/src/blue/blue_description/meshes:$GZ_SIM_RESOURCE_PATH
