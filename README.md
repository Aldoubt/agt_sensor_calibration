# agt_sensor_calibration

AgroTech multi-sensor calibration framework.

## Goal

Provide a reusable ROS2 framework for:

- BUNKER / mobile chassis odometry integration
- Livox MID360 extrinsic calibration
- IMU and future RTK-GPS integration
- automatic rosbag collection
- online/offline SE(3) extrinsic optimization
- calibration validation and export

## First milestone

small chassis + MID360 + IMU:

```
record bag -> estimate trajectories -> optimize lidar_to_base -> validate TF
```

## Architecture

```
agt_sensor_calibration
├── calibration_bringup
├── data_collection
├── extrinsic_calibration
├── optimization
├── validation
└── configs
```

ROS2 Humble target.

## Offline extrinsic calibration

The implemented C++ package reads the wheel and LIO Odometry topics directly
from a rosbag2 SQLite bag, synchronizes them by pose interpolation, and solves
the SE(3) motion-conjugacy problem with Ceres:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_extrinsic_calibration
source install/setup.bash
ros2 run agt_extrinsic_calibration calibrate_bag \
  --bag Benchmark-BAG-260725 --output calibration_output
```

Outputs include the two trajectory CSV files and
`calibration_output/results/mid360_to_base.yaml` plus
`optimization_report.md`. The bag profile in
`configs/bag_profiles/benchmark_bag_260725.yaml` documents the expected
LiDAR, IMU, wheel and LIO topics; this Phase 1 executable consumes the two
Odometry streams and leaves IMU/GPS factors for the next phase.
