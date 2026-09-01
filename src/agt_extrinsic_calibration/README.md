# agt_extrinsic_calibration

Offline ROS 2 Humble calibration of `base_link` to the Livox/LIO frame from two
`nav_msgs/msg/Odometry` streams. The reader uses rosbag2 SQLite storage and the
optimizer uses Eigen and Ceres. The SE(3) motion composition and interpolation
are kept in the core library so IMU/GPS factors can be added later.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agt_extrinsic_calibration
source install/setup.bash
```

## Run

```bash
ros2 run agt_extrinsic_calibration calibrate_bag \
  --bag Benchmark-BAG-260725 --output calibration_output
```

`--bag` is the rosbag2 directory (or URI), not the YAML profile name. The
default topics are the AGT wheel and LIO odometry topics. Results are written
to `wheel_trajectory.csv`, `lio_trajectory.csv`, and `results/` below the
selected output directory. Use `--sample-period` and `--min-motion` to tune
the synchronization and low-motion rejection.
