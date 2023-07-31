#!/usr/bin/env bash

# Add blue models and worlds
export GZ_SIM_RESOURCE_PATH=/workspaces/blue/blue_description/gazebo/models:/workspaces/blue/blue_description/gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Add blue meshes
export GZ_SIM_RESOURCE_PATH=/workspaces/blue/blue_description/meshes:$GZ_SIM_RESOURCE_PATH
