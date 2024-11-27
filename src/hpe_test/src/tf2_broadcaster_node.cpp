
#include <hpe_test/tf2_broadcaster_node.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2/time.h>

namespace hpe_test {

TF2BroadcasterNode::TF2BroadcasterNode() : Node("tf2_broadcaster") {

  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::sleep_for(std::chrono::seconds(5));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::string calibration_file = "./src/hpe_test/calibration/camera_poses.yaml";

  YAML::Node calibration_data = YAML::LoadFile(calibration_file);

  publishTransforms(calibration_data["poses"], "world");
}

void TF2BroadcasterNode::publishTransform(const std::string &child_frame,
                                          const YAML::Node &transform_info,
                                          const std::string &parent_frame) {
  geometry_msgs::msg::TransformStamped msg_;

  // Set header
  msg_.header.stamp = this->get_clock()->now();
  msg_.header.frame_id = parent_frame;

  // Set child frame
  msg_.child_frame_id = child_frame + "_optical_frame";

  // Set translation
  msg_.transform.translation.x =
      transform_info["translation"]["x"].as<double>();
  msg_.transform.translation.y =
      transform_info["translation"]["y"].as<double>();
  msg_.transform.translation.z =
      transform_info["translation"]["z"].as<double>();

  // Set rotation
  msg_.transform.rotation.x = transform_info["rotation"]["x"].as<double>();
  msg_.transform.rotation.y = transform_info["rotation"]["y"].as<double>();
  msg_.transform.rotation.z = transform_info["rotation"]["z"].as<double>();
  msg_.transform.rotation.w = transform_info["rotation"]["w"].as<double>();

  // Get real transforms

  std::string from = child_frame + "_color_optical_frame";
  std::string to = child_frame + "_link";
  RCLCPP_INFO(this->get_logger(), "Getting transforms from %s to %s",
              from.c_str(), to.c_str());
  geometry_msgs::msg::TransformStamped optical_to_link_msg =
      tf_buffer->lookupTransform(from, to, tf2::TimePointZero);

  tf2::Transform w_to_optical, optical_to_link, w_to_link;

  tf2::fromMsg(msg_.transform, w_to_optical);
  tf2::fromMsg(optical_to_link_msg.transform, optical_to_link);

  w_to_link = w_to_optical * optical_to_link;
  // Publish the transform
  geometry_msgs::msg::TransformStamped w_to_link_msg;
  w_to_link_msg.header.stamp = msg_.header.stamp;
  w_to_link_msg.header.frame_id = parent_frame;
  w_to_link_msg.child_frame_id = child_frame + "_link";

  w_to_link_msg.transform = tf2::toMsg(w_to_link);

  tf_broadcaster_->sendTransform(w_to_link_msg);
}

void TF2BroadcasterNode::publishTransforms(const YAML::Node &transforms,
                                           const std::string &parent_frame) {
  for (YAML::const_iterator it = transforms.begin(); it != transforms.end();
       ++it) {
    const std::string child_frame = it->first.as<std::string>();
    const YAML::Node &transform_info = it->second;
    publishTransform(child_frame, transform_info, parent_frame);
  }
}

} // namespace hpe_test
