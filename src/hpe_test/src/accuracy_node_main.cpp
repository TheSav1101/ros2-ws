#include <hpe_test/accuracy_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create and spin the TF2 broadcaster node
  rclcpp::spin(std::make_shared<hpe_test::AccuracyNode>());

  rclcpp::shutdown();
  return 0;
}
