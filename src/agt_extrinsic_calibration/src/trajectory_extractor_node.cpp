#include <rclcpp/rclcpp.hpp>

class TrajectoryExtractorNode : public rclcpp::Node
{
public:
  TrajectoryExtractorNode() : Node("trajectory_extractor_node")
  {
    RCLCPP_INFO(get_logger(), "AGT trajectory extractor initialized");
    RCLCPP_INFO(get_logger(), "Input profiles: wheel odom + lidar odom");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
