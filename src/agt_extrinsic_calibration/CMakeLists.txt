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
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(agt_calibration_core
  src/rosbag_reader.cpp
  src/trajectory.cpp
  src/optimizer.cpp
  src/prior_loader.cpp)
ament_target_dependencies(agt_calibration_core rclcpp rosbag2_cpp nav_msgs)
target_link_libraries(agt_calibration_core Eigen3::Eigen Ceres::ceres yaml-cpp)
target_compile_features(agt_calibration_core PUBLIC cxx_std_17)

add_executable(calibrate_bag src/calibrate_bag.cpp)
target_link_libraries(calibrate_bag agt_calibration_core)
ament_target_dependencies(calibrate_bag rclcpp)

add_executable(trajectory_extractor_node src/trajectory_extractor_node.cpp)
ament_target_dependencies(trajectory_extractor_node
  rclcpp rosbag2_cpp nav_msgs sensor_msgs tf2 tf2_ros)
target_compile_features(trajectory_extractor_node PRIVATE cxx_std_17)

install(TARGETS agt_calibration_core calibrate_bag trajectory_extractor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory agt_calibration_core)
  ament_target_dependencies(test_trajectory ament_index_cpp)
endif()

ament_package()
