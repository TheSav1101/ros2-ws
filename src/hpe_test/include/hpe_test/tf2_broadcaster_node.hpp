#ifndef HPE_TEST__TF2_BROADCASTER_NODE_HPP_
#define HPE_TEST__TF2_BROADCASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace hpe_test {
class TF2BroadcasterNode : public rclcpp::Node {
public:
  TF2BroadcasterNode();

private:
  void publishTransforms(const YAML::Node &transforms,
                         const std::string &parent_frame);

  void publishTransform(const std::string &child_frame,
                        const YAML::Node &transform_info,
                        const std::string &parent_frame);

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};
} // namespace hpe_test

#endif
