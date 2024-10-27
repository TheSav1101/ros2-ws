#include <hpe_test/slave_node_single.hpp>

int main(int argc, char *argv[]) {
	if (argc < 5 || argc >= 6) {
		std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model>\n";
		return 1;
	}
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<hpe_test::SlaveNodeSingle>(argv[1], argv[2], atoi(argv[3])));
	
	rclcpp::shutdown();
	
	
	return 0;
}