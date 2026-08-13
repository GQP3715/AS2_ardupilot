#!/bin/bash

# Launch Gazebo and the ArduPilot SITL instances described in
# sitl_config/ardupilot/world.yaml

config_file="sitl_config/ardupilot/world.yaml"

drones_namespace_comma=$(python3 utils/get_drones.py -p ${config_file} --sep ',')

echo "Launching ArduPilot SITL simulation for drones: ${drones_namespace_comma}"

tmuxinator start -n ardupilot_sitl -p tmuxinator/sitl_simulation.yaml \
  drone_namespace=${drones_namespace_comma} \
  config_file=${config_file} \
  wait

tmux attach-session -t ardupilot_sitl