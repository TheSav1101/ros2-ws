#ifndef HPE_TEST__MASTER_NODE_SINGLE_HPP_
#define HPE_TEST__MASTER_NODE_SINGLE_HPP_

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hpe_test {
class AccuracyNode : public rclcpp::Node {
public:
  AccuracyNode();
  ~AccuracyNode();

private:
  void network_callback(const visualization_msgs::msg::MarkerArray &msg);
  void metrabs_callback(const visualization_msgs::msg::MarkerArray &msg);
  double compute_distance(const geometry_msgs::msg::Point &netw,
                          const std::string &netw_frame,
                          const geometry_msgs::msg::Point &metr,
                          const std::string &metr_frame);

  const double MIN_HIGH_CONF = 0.5;

  const double DISTANCE_THRESHOLD = 0.2;

  visualization_msgs::msg::MarkerArray last_network;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      metrabs_markers_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      network_markers_subscription_;

  size_t number_of_joints_tested;
  double max_pck;
  size_t pck;
  size_t number_of_joints_tested_high_conf;
  double max_pck_high_conf;
  size_t pck_high_conf;

  double mpjme;
  double hc_mpjme;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};
} // namespace hpe_test

#endif
