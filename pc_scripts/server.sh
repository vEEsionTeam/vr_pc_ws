#!/bin/bash

# Default values
SERVER_IP="192.168.x.x"
IMG_ENABLE=1
PATH_ENABLE=0
POINTS_ENABLE=0

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --ip)
      SERVER_IP="$2"
      shift 2
      ;;
    --img)
      IMG_ENABLE="$2"
      shift 2
      ;;
    --path)
      PATH_ENABLE="$2"
      shift 2
      ;;
    --points)
      POINTS_ENABLE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./comm.sh [--ip <ip>] [--img <0|1>] [--path <0|1>] [--points <0|1>]"
      exit 1
      ;;
  esac
done

cd ~/vr_pc_ws || exit
source install/setup.bash

ros2 run tf_server tf_server2 \
  --ros-args \
  -p img_enable:="${IMG_ENABLE}" \
  -p path_enable:="${PATH_ENABLE}" \
  -p points_enable:="${POINTS_ENABLE}" \
  -p server_ip:="${SERVER_IP}"
