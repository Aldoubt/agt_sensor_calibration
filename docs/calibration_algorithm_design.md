# AGT Extrinsic Calibration Algorithm Design

## Goal

Estimate:

```
T_base_lidar
```

from recorded robot motion.

## Current input

Benchmark-BAG-260725:

- /agt/sensors/lidar/custom
- /agt/sensors/imu/data
- /agt/chassis/odometry
- /agt/mapping/odometry
- /tf
- /tf_static

## Pipeline

```
rosbag2
 |
 trajectory extraction
 |
 wheel trajectory
 lidar trajectory
 |
 SE(3) optimization
 |
 extrinsic yaml
```

## Optimization model

The unknown parameter is the rigid transform:

```
T_base_lidar
```

The optimizer will use:

- wheel odometry motion constraint
- lidar odometry motion constraint
- optional IMU rotation constraint

Future extensions:

- GPS to base calibration
- time offset optimization
- online calibration
