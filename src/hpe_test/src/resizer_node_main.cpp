#include <hpe_test/resizer_node.hpp>

int main(int argc, char *argv[]) {
	if (argc < 5) {
		std::cout << "Usage: ros2 run hpe_test image_resizer <name> <raw_topic> <model> <detection_model> <optional: starting_workers>\n";
		return 1;
	}
	int starting_workers = 2;
	if(argc == 6){
		starting_workers = atoi(argv[4]);
	}else if(argc > 6){
		return 1;
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<hpe_test::ResizerNode>(argv[1], argv[2], atoi(argv[3]), atoi(argv[4]), starting_workers));
	rclcpp::shutdown();
	return 0;
}