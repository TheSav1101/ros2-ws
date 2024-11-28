#include <hpe_test/master_node_single.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
  if (signal == SIGTERM || signal == SIGINT) {
    keep_running.store(false);
  }
}

int main(int argc, char *argv[]) {
  std::string name = "master";

  if (argc >= 2) {
    name = argv[1];
  }

  rclcpp::init(argc, argv);
  try {
    {
      rclcpp::executors::MultiThreadedExecutor executor;
      auto node = std::make_shared<hpe_test::MasterNodeSingle>(name, &executor);
      executor.add_node(node);
      rclcpp::Rate rate(15);
      while (keep_running.load() && rclcpp::ok()) {
        executor.spin_some();
        rate.sleep();
      }
      node->shutdown();
      executor.remove_node(node);
    }
    rclcpp::shutdown();
    std::cout << "ROS2 shutdown complete" << std::endl;
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "Error during shutdown: " << e.what() << std::endl;
  }

  return 0;
}
