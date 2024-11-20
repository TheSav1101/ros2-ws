#include <hpe_test/master_node.hpp>

int main(int argc, char *argv[]) {
  std::string name = "master";

  if (argc >= 2) {
    name = argv[1];
  }

  rclcpp::init(argc, argv);
  try {
    {
      rclcpp::executors::MultiThreadedExecutor executor;
      auto node = std::make_shared<hpe_test::MasterNode>(name);
      executor.add_node(node);
      executor.spin();
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
