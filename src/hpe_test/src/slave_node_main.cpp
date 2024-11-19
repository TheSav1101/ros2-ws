#include <hpe_test/slave_node.hpp>

int main(int argc, char *argv[]) {
	if (argc < 5) {
		std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model> <detection_model> <optional: starting_workers> <optional: calibration topic>\n";
		return 1;
	}
	int starting_workers = 3;
	std::string calibration_topic = "";
	if(argc >= 6){
		starting_workers = atoi(argv[5]);
	}
	if(argc >= 7){
		calibration_topic = argv[6];
	
	}else if(argc > 7){
		return 1;
	}
	rclcpp::init(argc, argv);
	try {
		{
			auto executor = rclcpp::executors::MultiThreadedExecutor();
			auto node = std::make_shared<hpe_test::SlaveNode>(&executor, argv[1], argv[2], atoi(argv[3]), atoi(argv[4]), starting_workers, calibration_topic);
			executor.add_node(node);
			executor.spin();
			executor.cancel();
			node->shutdown();	
		}
		rclcpp::shutdown();
		std::cout << "ROS2 shutdown complete" << std::endl;
	} catch (const rclcpp::exceptions::RCLError & e) {
		std::cerr << "Error during shutdown: " << e.what() << std::endl;
	}
	
	return 0;
}