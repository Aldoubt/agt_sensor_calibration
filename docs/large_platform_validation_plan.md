# Large platform validation plan

## Goal

Validate the wheel + LIO extrinsic calibration pipeline on the BUNKER large
mobile platform before adding IMU/GPS joint factors.

## Data requirements

Required topics:

- `/agt/chassis/odometry`
- `/agt/mapping/odometry`
- `/agt/sensors/imu/data`
- `/tf`
- `/tf_static`

Optional:

- RTK GPS

## Motion excitation

The calibration run should include:

1. Straight motion
2. Left/right turns
3. Figure-eight motion
4. In-place rotation if possible
5. Different speeds

Avoid using only straight-line segments because yaw and lateral lever arm
parameters become weakly observable.

## Validation output

The calibration report should compare:

- prior extrinsic
- optimized extrinsic
- residual before/after
- physical plausibility checks

Reject solutions with large translation changes without a corresponding
physical explanation.
