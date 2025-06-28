#!/bin/bash

# ----------------- Defaults -----------------
WORLD="Office.world"     # default world file

# -------------- Parse options --------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)
      WORLD="$2"           # override default
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./gazebo.sh [--world <world_file.world>]"
      exit 1
      ;;
  esac
done

# -------------- Run Gazebo ------------------
cd ~/vr_pc_ws || exit
source install/setup.bash
ros2 launch gazebo_sim gazebo.launch.py world:="${WORLD}"

