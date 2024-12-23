#include <atomic>
#include <csignal>
#include <cstdlib>
#include <hpe_test/slave_node.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
  if (signal == SIGTERM || signal == SIGINT) {
    keep_running.store(false);
  }
}

int main(int argc, char *argv[]) {
  if (argc < 5) {
    std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model> "
                 "<detection_model> <optional: starting_workers> <optional: "
                 "calibration topic> <optional GPU acceleration = 0 or 1>\n";
    return 1;
  }
  int starting_workers = 3;
  int gpu = 0;
  std::string calibration_topic = "";
  if (argc >= 6) {
    starting_workers = atoi(argv[5]);
  }
  if (argc >= 7) {
    calibration_topic = argv[6];
  }
  if (argc >= 8) {
    gpu = atoi(argv[7]);

  } else if (argc > 8) {
    return 1;
  }
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    {
      auto options = rclcpp::ExecutorOptions();
      auto executor = rclcpp::executors::MultiThreadedExecutor(options, 4);
      auto node = std::make_shared<hpe_test::SlaveNode>(
          &executor, argv[1], argv[2], atoi(argv[3]), atoi(argv[4]),
          starting_workers, calibration_topic, gpu);
      executor.add_node(node);

      rclcpp::Rate rate(30);
      while (keep_running.load() && rclcpp::ok()) {
        executor.spin_some();
        rate.sleep();
      }
      node->shutdown();
      executor.spin_some();
      executor.cancel();
    }
    rclcpp::shutdown();
    std::cout << "ROS2 shutdown complete" << std::endl;
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "Error during shutdown: " << e.what() << std::endl;
  }

  return 0;
}
