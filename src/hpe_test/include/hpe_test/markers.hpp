#ifndef HPE_TEST__MARKERS_FAKE_HPP_
#define HPE_TEST__MARKERS_FAKE_HPP_

#include "visualization_msgs/msg/marker_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hpe_test {

class MarkersNode : public rclcpp::Node {
public:
  MarkersNode();
  ~MarkersNode();

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_sub_;

  void callback(const visualization_msgs::msg::MarkerArray &msg);
};
} // namespace hpe_test
#endif
