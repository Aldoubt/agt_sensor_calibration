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
