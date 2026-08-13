#!/bin/bash
# Launch the Gazebo world used by the ArduPilot SITL instances.
#
# Requires the ArduPilot Gazebo plugin (https://github.com/ArduPilot/ardupilot_gazebo)
# to be built and the following environment to be exported:
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
#   export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

set -euo pipefail

CONFIG_FILE=${1:-sitl_config/ardupilot/world.yaml}

WORLD=$(python3 -c "import yaml,sys;print(yaml.safe_load(open('${CONFIG_FILE}'))['world'])")

echo "Launching Gazebo world: ${WORLD}.sdf"
exec gz sim -v4 -r "${WORLD}.sdf"