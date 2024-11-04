#include <hpe_test/slave_node_single.hpp>

int main(int argc, char *argv[]) {
	if (argc < 4 || argc >= 6) {
		std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model> <optional: calibration topic>\n";
		return 1;
	}

	std::string calibration_topic = "";

	if(argc >= 5)
		calibration_topic = argv[4];

	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<hpe_test::SlaveNodeSingle>(argv[1], argv[2], atoi(argv[3]), calibration_topic));
	
	rclcpp::shutdown();
	
	
	return 0;
}