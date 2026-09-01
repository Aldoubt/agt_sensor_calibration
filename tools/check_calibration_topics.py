#!/usr/bin/env python3
"""Check whether a rosbag contains required calibration topics.

Designed for Benchmark-BAG and future small chassis + MID360 bags.
"""

REQUIRED = {
    "/agt/sensors/lidar/custom": "Livox MID360 raw lidar",
    "/agt/sensors/imu/data": "IMU",
    "/agt/chassis/odometry": "wheel odometry",
}


def check(topic_names):
    missing = []
    for topic, desc in REQUIRED.items():
        if topic not in topic_names:
            missing.append((topic, desc))
    return missing


if __name__ == "__main__":
    print("Use rosbag2_py reader to pass topic names into check().")
