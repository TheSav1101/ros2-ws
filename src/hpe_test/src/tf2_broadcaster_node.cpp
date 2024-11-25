
#include <hpe_test/tf2_broadcaster_node.hpp>

namespace hpe_test {

TF2BroadcasterNode::TF2BroadcasterNode() : Node("tf2_broadcaster") {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  old_msgs = {};

  std::string calibration_file = "./src/hpe_test/calibration/camera_poses.yaml";

  YAML::Node calibration_data = YAML::LoadFile(calibration_file);

  publishTransforms(calibration_data["poses"], "world");
  // publishTransforms(calibration_data["inverse_poses"], "world");

  auto timer_ =
      this->create_wall_timer(std::chrono::milliseconds(5),
                              std::bind(&TF2BroadcasterNode::resend_all, this));
}

void TF2BroadcasterNode::resend_all() {
  for (auto msg : old_msgs) {
    msg.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(msg);
  }
}

void TF2BroadcasterNode::publishTransform(const std::string &child_frame,
                                          const YAML::Node &transform_info,
                                          const std::string &parent_frame) {
  geometry_msgs::msg::TransformStamped msg;

  // Set header
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = parent_frame;

  // Set child frame
  msg.child_frame_id = child_frame;

  // Set translation
  msg.transform.translation.x = transform_info["translation"]["x"].as<double>();
  msg.transform.translation.y = transform_info["translation"]["y"].as<double>();
  msg.transform.translation.z = transform_info["translation"]["z"].as<double>();

  // Set rotation
  msg.transform.rotation.x = transform_info["rotation"]["x"].as<double>();
  msg.transform.rotation.y = transform_info["rotation"]["y"].as<double>();
  msg.transform.rotation.z = transform_info["rotation"]["z"].as<double>();
  msg.transform.rotation.w = transform_info["rotation"]["w"].as<double>();

  // Publish the transform

  old_msgs.push_back(msg);
  tf_broadcaster_->sendTransform(msg);
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
