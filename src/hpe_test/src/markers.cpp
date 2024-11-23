#include "visualization_msgs/msg/marker_array.hpp"
#include <hpe_test/markers.hpp>
#include <rclcpp/logging.hpp>

using std::placeholders::_1;

namespace hpe_test {

MarkersNode::MarkersNode() : rclcpp::Node("markers") {
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "tracker/skeleton_markers_array_now", 10);
  marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "tracker/skeleton_markers_array", 10,
      std::bind(&MarkersNode::callback, this, _1));
}
MarkersNode::~MarkersNode() {}

void MarkersNode::callback(const visualization_msgs::msg::MarkerArray &msg) {
  auto msg2 = visualization_msgs::msg::MarkerArray();
  int i = 0;
  for (auto marker : msg.markers) {
    marker.header.stamp = this->get_clock()->now();
    msg2.markers.push_back(marker);
    i++;
  }
  marker_pub_->publish(msg2);
  RCLCPP_INFO(this->get_logger(), "Done %d", i);
}
} // namespace hpe_test

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hpe_test::MarkersNode>());
  rclcpp::shutdown();
  return 0;
}
