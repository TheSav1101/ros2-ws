#include <hpe_test/slave_node_single.hpp>

int main(int argc, char *argv[]) {
  if (argc < 4 || argc >= 7) {
    std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model> "
                 "<optional: calibration topic> <optional: optical frame>\n";
    return 1;
  }

  std::string calibration_topic = "";
  std::string optical_frame = "";

  if (argc >= 5)
    calibration_topic = argv[4];

  if (argc >= 6)
    optical_frame = argv[5];

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hpe_test::SlaveNodeSingle>(
      argv[1], argv[2], atoi(argv[3]), calibration_topic, optical_frame));

  rclcpp::shutdown();

  return 0;
}
