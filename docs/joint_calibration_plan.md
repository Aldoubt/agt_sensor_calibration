# Joint LiDAR IMU Wheel Calibration Plan

## Goal

Upgrade Phase 1 wheel-LIO motion calibration into a multi-sensor extrinsic calibration framework.

Target parameters:

- T_base_lidar
- T_lidar_imu
- optional time offset
- future T_base_gps

## Extrinsic Prior Definition

The prior is NOT generated from odometry. It represents a physical mounting estimate:

T_base_lidar = transform from robot base_link frame to MID360 lidar frame.

Example:

base_link -> mid360_link

The prior can come from:

1. CAD measurement
2. manual measurement with ruler/caliper
3. existing tf_static
4. previous calibration result

The bag can validate or refine the prior, but cannot reliably recover all 6 DoF without sufficient motion excitation.

## Current bag usage

Benchmark-BAG-260725 can provide:

- wheel odometry
- LIO odometry
- IMU
- TF

Phase 1 uses wheel and LIO odometry.

Phase 2 adds:

- IMU preintegration factor
- gravity alignment
- time offset optimization
- prior constrained SE(3) optimization

## Constraints

For planar agricultural robots:

- z translation is weakly observable
- roll/pitch require sufficient terrain variation
- straight line motion is insufficient
- rotation excitation is required

Calibration experiments should include:

- straight motion
- left/right turns
- figure-eight trajectory
- in-place rotation if possible
