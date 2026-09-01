# Benchmark prior validation workflow

## Goal

Use the static transform or CAD measurement as an initial guess for
`T_base_mid360` before Ceres optimization.

The direction is:

```
base_link -> mid360_link
```

The prior is not recovered from the bag. The bag provides motion constraints:

```
wheel odometry
        +
LIO odometry
        |
        v
SE(3) motion optimization
        |
        v
refined base_link -> mid360_link
```

## Validation steps

1. Extract `/tf_static` from rosbag if available.
2. Compare the transform with CAD measurement.
3. Use it as an optimization prior.
4. Reject solutions with excessive deviation.

The prior factor prevents degenerate planar-motion solutions such as large
lateral offsets with low residual error.
