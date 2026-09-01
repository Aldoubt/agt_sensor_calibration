# Benchmark-BAG-260725 calibration pipeline

## Available data

The bag already contains the core data required for the first calibration stage:

- Livox MID360 raw scan
- IMU
- wheel odometry
- mapping odometry
- TF

Therefore no new data collection is required for offline development.

## First stage

Use:

```
/agt/sensors/lidar/custom
/agt/sensors/imu/data
/agt/chassis/odometry
```

to estimate:

```
T_base_lidar
```

## Later extension

Add:

```
/gps/fix
/camera/image
```

without changing the calibration interface.
