#ifndef HPE_TEST__TF2_BROADCASTER_NODE_HPP_
#define HPE_TEST__TF2_BROADCASTER_NODE_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace hpe_test {
class TF2BroadcasterNode : public rclcpp::Node {
public:
  TF2BroadcasterNode();

private:
  void resend_all();
  void publishTransforms(const YAML::Node &transforms,
                         const std::string &parent_frame);

  void publishTransform(const std::string &child_frame,
                        const YAML::Node &transform_info,
                        const std::string &parent_frame);

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<geometry_msgs::msg::TransformStamped> old_msgs;
};
} // namespace hpe_test

#endif
