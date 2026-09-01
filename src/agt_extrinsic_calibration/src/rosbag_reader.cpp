#include "agt_extrinsic_calibration/rosbag_reader.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace agt_extrinsic_calibration {

Trajectory RosbagReader::read_odometry(const std::string &bag_uri, const std::string &topic) const {
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_uri;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
  reader.open(storage_options, converter_options);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);
  rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
  Trajectory result;
  while (reader.has_next()) {
    auto message = reader.read_next();
    if (message->topic_name != topic) continue;
    nav_msgs::msg::Odometry odom;
    rclcpp::SerializedMessage serialized(*message->serialized_data);
    serialization.deserialize_message(&serialized, &odom);
    PoseStamped pose;
    pose.timestamp = static_cast<double>(message->time_stamp) * 1e-9;
    if (odom.header.stamp.sec != 0 || odom.header.stamp.nanosec != 0) {
      pose.timestamp = static_cast<double>(odom.header.stamp.sec) + odom.header.stamp.nanosec * 1e-9;
    }
    pose.position = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z};
    pose.orientation = Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                           odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).normalized();
    result.push_back(pose);
  }
  if (result.empty()) throw std::runtime_error("No messages found on topic " + topic);
  return result;
}

void RosbagReader::write_csv(const Trajectory &trajectory, const std::string &filename) const {
  std::ofstream out(filename);
  if (!out) throw std::runtime_error("Cannot write " + filename);
  out << "timestamp,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
  for (const auto &p : trajectory) out << p.timestamp << ',' << p.position.x() << ',' << p.position.y() << ','
      << p.position.z() << ',' << p.orientation.x() << ',' << p.orientation.y() << ',' << p.orientation.z() << ','
      << p.orientation.w() << '\n';
}
}  // namespace agt_extrinsic_calibration
