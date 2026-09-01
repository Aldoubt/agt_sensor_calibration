# Benchmark: mapping_20260719_172810

Experiment date: 2026-09-01. The source bag is
`/home/yangxuan/agt_navigation_v2/runtime/rosbag/mapping_20260719_172810`.
The bag duration is 807.45 s. Calibration used wheel odometry
`/agt/chassis/odometry` and LIO odometry `/agt/mapping/odometry`, sampled at
0.1 s with the default low-motion threshold.

## Window comparison

| Relative window (s) | Motion pairs | Initial RMSE | Final RMSE | Iterations |
|---:|---:|---:|---:|---:|
| 250–400 | 982 | 0.00517219 | 0.00396708 | 3 |
| 400–600 | 1998 | 0.0211121 | 0.0203916 | 5 |
| 600–780 | 1798 | 0.0360041 | 0.00166372 | 9 |

The 250–400 s result is the current candidate. The 600–780 s fit has a much
lower residual but an implausible 29.37 m lateral translation and is treated as
degenerate. All three fits have z=0, which is expected to be unobservable from
the current mostly planar motion and is not evidence that the physical z offset
is zero.

## Reproduction

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agt_extrinsic_calibration calibrate_bag \
  --bag /home/yangxuan/agt_navigation_v2/runtime/rosbag/mapping_20260719_172810 \
  --start-offset 250 --duration 150 \
  --output /tmp/agt_250_400
```

The complete CSV extracts remain outside git under `/tmp/agt_250_400`,
`/tmp/agt_400_600_repeat`, and `/tmp/agt_600_780` because they are generated
artifacts and are not suitable for repository storage.
