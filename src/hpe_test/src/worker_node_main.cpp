#include <hpe_test/worker_node.hpp>

int main(int argc, char *argv[]) {
	if (argc != 3) {
		std::cout << "Usage: ros2 run hpe_test worker <name> <model_path>\n";
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<hpe_test::WorkerNode>(argv[1], argv[2]));
	rclcpp::shutdown();
	return 0;
}