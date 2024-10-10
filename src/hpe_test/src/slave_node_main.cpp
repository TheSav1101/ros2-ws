#include <hpe_test/slave_node.hpp>

int main(int argc, char *argv[]) {
	if (argc < 5) {
		std::cout << "Usage: ros2 run hpe_test slave <name> <raw_topic> <model> <detection_model> <optional: starting_workers>\n";
		return 1;
	}
	int starting_workers = 3;
	if(argc == 6){
		starting_workers = atoi(argv[5]);
	}else if(argc > 6){
		return 1;
	}
	rclcpp::init(argc, argv);
	try {
		{
			rclcpp::executors::MultiThreadedExecutor executor;
			auto node = std::make_shared<hpe_test::SlaveNode>(argv[1], argv[2], atoi(argv[3]), atoi(argv[4]), starting_workers);
			executor.add_node(node);
			executor.spin();
			node->shutdown();
			executor.remove_node(node);
		}
		rclcpp::shutdown();
		std::cout << "ROS2 shutdown complete" << std::endl;
	} catch (const rclcpp::exceptions::RCLError & e) {
		std::cerr << "Error during shutdown: " << e.what() << std::endl;
	}
	
	return 0;
}