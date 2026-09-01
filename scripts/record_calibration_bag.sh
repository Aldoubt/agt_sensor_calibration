#!/bin/bash

set -e

OUT=${1:-calibration_bag}
mkdir -p ${OUT}

ros2 bag record \
  -o ${OUT} \
  /livox/lidar \
  /livox/imu \
  /wheel/odom \
  /tf \
  /tf_static
