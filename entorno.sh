#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/grupo2_kuka/install/setup.bash" ]; then
  source "$HOME/grupo2_kuka/install/setup.bash"
fi
