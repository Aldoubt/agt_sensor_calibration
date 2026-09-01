cmake_minimum_required(VERSION 3.8)
project(agt_extrinsic_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_extractor_node src/trajectory_extractor_node.cpp)
ament_target_dependencies(trajectory_extractor_node
  rclcpp rosbag2_cpp nav_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS trajectory_extractor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
