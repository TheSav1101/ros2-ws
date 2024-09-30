#include <hpe_test/webcam_node.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	if (argc == 2) {
		rclcpp::spin(std::make_shared<hpe_test::WebcamNode>(argv[1]));
	}else{
		std::cout << "Usage: ros2 run hpe_test webcam <name>";
	}
	rclcpp::shutdown();
	return 0;
}