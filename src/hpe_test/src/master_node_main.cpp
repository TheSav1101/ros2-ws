#include <hpe_test/master_node.hpp>

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	if (argc < 2) {
		rclcpp::spin(std::make_shared<hpe_test::MasterNode>("master"));

	}else{
		rclcpp::spin(std::make_shared<hpe_test::MasterNode>(argv[1]));
	}
	rclcpp::shutdown();
	return 0;
}