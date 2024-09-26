#include <hpe_test/visualizer_node.hpp>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ros2 run hpe_test visualizer <name>, where <name> is the same name used for an active slave\n";
        return 0;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hpe_test::VisualizerNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
