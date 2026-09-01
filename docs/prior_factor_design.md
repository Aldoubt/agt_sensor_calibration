# Extrinsic Prior Factor Design

## Target transform

The calibration target is:

```
base_link -> mid360_link
```

or mathematically:

```
T_base_mid360
```

The prior represents an approximate physical mounting relationship.

## Sources of prior

Recommended priority:

1. URDF / robot_description static transform
2. /tf_static recorded in rosbag
3. CAD measurement
4. Manual measurement with tape/caliper

## Can rosbag motion recover it?

No. A bag containing only:

- wheel odometry
- LIO odometry

provides relative motion constraints:

```
Delta T_wheel
Delta T_lio
```

It can refine an initial transform, but cannot reliably provide the absolute sensor mounting transform.

## Optimization model

Future optimizer will use:

```
total_cost = motion_cost + lambda * prior_cost
```

where:

```
prior_cost = Log(T_prior^-1 * T_est)^2
```

This prevents physically impossible solutions such as meter-scale lateral offsets.

## Future extensions

The same mechanism will support:

- base_link -> imu_link
- lidar_link -> imu_link
- base_link -> gps_link
- time offset priors
