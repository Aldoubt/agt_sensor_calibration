# Extrinsic Calibration

Target:

```
T_base_lidar
```

## Planned pipeline

1. Read wheel odometry trajectory.
2. Read LiDAR odometry trajectory.
3. Build SE(3) residual.
4. Optimize with Ceres Solver.
5. Export yaml calibration result.

Future extensions:

- GPS to base calibration
- camera to lidar calibration
- time offset estimation
