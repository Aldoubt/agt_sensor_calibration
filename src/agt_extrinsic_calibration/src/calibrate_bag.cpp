#include "agt_extrinsic_calibration/optimizer.hpp"
#include "agt_extrinsic_calibration/rosbag_reader.hpp"
#include "agt_extrinsic_calibration/trajectory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <iostream>

using namespace agt_extrinsic_calibration;

int main(int argc, char **argv) {
  std::string bag, output = ".", wheel_topic = "/agt/chassis/odometry", lio_topic = "/agt/mapping/odometry";
  double sample_period = 0.1, min_motion = 0.001, start_offset = 0.0, duration = 0.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    auto value = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
      return argv[++i];
    };
    if (a == "--bag") bag = value(a);
    else if (a == "--output") output = value(a);
    else if (a == "--sample-period") sample_period = std::stod(value(a));
    else if (a == "--min-motion") min_motion = std::stod(value(a));
    else if (a == "--start-offset") start_offset = std::stod(value(a));
    else if (a == "--duration") duration = std::stod(value(a));
    else if (a == "--help" || a == "-h") {
      std::cout << "Usage: calibrate_bag --bag BAG_URI [--output DIR] [--sample-period SEC] "
                   "[--start-offset SEC --duration SEC]\n"; return 0;
    }
  }
  if (bag.empty()) { std::cerr << "--bag is required\n"; return 2; }
  rclcpp::init(argc, argv);
  try {
    std::filesystem::create_directories(output + "/results");
    RosbagReader reader;
    auto wheel = reader.read_odometry(bag, wheel_topic);
    auto lio = reader.read_odometry(bag, lio_topic);
    if (start_offset > 0.0 || duration > 0.0) {
      const double common_start = std::max(wheel.front().timestamp, lio.front().timestamp);
      const double begin = common_start + std::max(0.0, start_offset);
      const double end = duration > 0.0 ? begin + duration : std::min(wheel.back().timestamp, lio.back().timestamp);
      auto crop = [begin, end](const Trajectory &input) {
        Trajectory result;
        for (const auto &p : input) if (p.timestamp >= begin && p.timestamp <= end) result.push_back(p);
        return result;
      };
      wheel = crop(wheel); lio = crop(lio);
      if (wheel.size() < 2 || lio.size() < 2) throw std::runtime_error("Selected time window has too few poses");
      std::cout << "selected window: " << (begin - common_start) << "s to " << (end - common_start) << "s\n";
    }
    reader.write_csv(wheel, output + "/wheel_trajectory.csv");
    reader.write_csv(lio, output + "/lio_trajectory.csv");
    auto pairs = make_motion_pairs(wheel, lio, sample_period, min_motion);
    auto report = optimize_extrinsic(pairs);
    write_result(report, output + "/results/mid360_to_base.yaml", output + "/results/optimization_report.md");
    std::cout << "wheel poses: " << wheel.size() << ", lio poses: " << lio.size()
              << ", motion pairs: " << pairs.size() << "\n"
              << "initial RMSE: " << report.initial_rmse << ", final RMSE: " << report.final_rmse << "\n";
    rclcpp::shutdown(); return report.success ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << "Calibration failed: " << e.what() << "\n";
    rclcpp::shutdown(); return 1;
  }
}
