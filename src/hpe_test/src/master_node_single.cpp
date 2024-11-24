#include "Eigen/Core"
#include "hpe_test/visualizer_node.hpp"
#include <hpe_test/master_node_single.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hpe_test {

MasterNodeSingle::MasterNodeSingle(
    std::string name, rclcpp::executors::MultiThreadedExecutor *executor)
    : Node(name) {
  avg_delay = 0.0;
  delay_window = 0;
  last_marker_count_ = 0;

  executor_ = executor;

  filtered_feedbacks = {};
  camera_indices = {};

  subscribers_ = {};
  slaves_feedback_ = {};
  scan_for_slaves();
  scanner_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&MasterNodeSingle::scan_for_slaves, this));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "master_node_output", 10);

  loop_thread = std::thread(&MasterNodeSingle::loop, this);
  // loop_thread.detach();
}

void MasterNodeSingle::shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  running_ = false;

  if (loop_thread.joinable()) {
    loop_thread.join();
    RCLCPP_INFO(this->get_logger(), "Loop thread joined...");
  }
  RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay);
}

MasterNodeSingle::~MasterNodeSingle() {
  running_ = false;
  if (loop_thread.joinable()) {
    loop_thread.join();
    RCLCPP_INFO(this->get_logger(), "Loop thread joined...");
  }
  RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay);
}

void MasterNodeSingle::callback(const hpe_msgs::msg::Slave &msg, int index) {
  slaves_feedback_[index] = msg;
  RCLCPP_INFO(this->get_logger(), "Recived from %d", index);
}

void MasterNodeSingle::loop() {
  running_ = true;
  rclcpp::Rate loop_rate(60);
  while (running_) {
    auto start = this->get_clock()->now();

    // Filter feedbacks by time
    filtered_feedbacks = {};
    camera_indices = {};
    filterFeedbacks();

    triangulate_all();

    double delay = (this->get_clock()->now() - start).seconds();
    avg_delay = (avg_delay * delay_window + delay);
    delay_window++;
    avg_delay /= delay_window;
    loop_rate.sleep();
  }
}

void MasterNodeSingle::triangulate_all() {

  if (filtered_feedbacks.size() <= 1) {
    RCLCPP_WARN(this->get_logger(), "No good slave respones yet");
    return;
  }

  std::vector<Eigen::Vector3f> joints3d = {};
  std::vector<float> avg_conf = {};

  for (int i = 0; i < max_joints; i++) {
    std::vector<float> confidences = {};
    float avg_conf_ = 0.0;

    for (auto f : filtered_feedbacks) {
      confidences.push_back(f.all_joints[0].confidence[i]);
      avg_conf_ += f.all_joints[0].confidence[i];
    }

    avg_conf_ /= filtered_feedbacks.size();

    buildAB_lists(i);

    joints3d.push_back(iterateWithWeights(confidences));

    avg_conf.push_back(avg_conf_);
  }

  clear_markers();
  visualize_3d(joints3d, avg_conf);
}

void MasterNodeSingle::visualize_3d(std::vector<Eigen::Vector3f> &joints,
                                    std::vector<float> &avg_conf) {
  visualization_msgs::msg::MarkerArray marker_array;

  // RCLCPP_WARN(this->get_logger(), "---------------------------");

  for (size_t i = 0; i < joints.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "joints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = joints[i].x();
    marker.pose.position.y = joints[i].y();
    marker.pose.position.z = joints[i].z();
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0f - avg_conf[i];
    color.g = avg_conf[i];
    color.b = 0.0f;
    color.a = 1.0f;
    marker.color = color;

    marker_array.markers.push_back(marker);
    // RCLCPP_INFO(this->get_logger(), "Joint %ld: x=%f, y=%f, z=%f", i,
    // joints[i].x(), joints[i].y(),joints[i].z());
  }

  // RCLCPP_WARN(this->get_logger(), "---------------------------");

  // edges
  std::vector<std::pair<int, int>> keypoint_edges = {
      {0, 1},  {0, 2},   {1, 3},   {2, 4},   {0, 5},   {0, 6},
      {5, 7},  {7, 9},   {6, 8},   {8, 10},  {5, 6},   {5, 11},
      {6, 12}, {11, 12}, {11, 13}, {13, 15}, {12, 14}, {14, 16}};

  int edge_marker_id = joints.size();
  for (const auto &edge : keypoint_edges) {
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "world";
    edge_marker.header.stamp = this->get_clock()->now();
    edge_marker.ns = "edges";
    edge_marker.id = edge_marker_id++;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.scale.x = 0.02; // Line thickness

    // Add the two points that define the edge
    geometry_msgs::msg::Point p1, p2;
    p1.x = joints[edge.first].x();
    p1.y = joints[edge.first].y();
    p1.z = joints[edge.first].z();
    p2.x = joints[edge.second].x();
    p2.y = joints[edge.second].y();
    p2.z = joints[edge.second].z();

    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);

    // Compute the average confidence for the edge
    float conf1 = avg_conf[edge.first];
    float conf2 = avg_conf[edge.second];
    float avg_edge_conf = (conf1 + conf2) / 2.0f;

    // Assign color based on average confidence
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0f - avg_edge_conf;
    color.g = avg_edge_conf;
    color.b = 0.0f;
    color.a = 1.0f;
    edge_marker.color = color;

    marker_array.markers.push_back(edge_marker);
  }

  marker_pub_->publish(marker_array);
  last_marker_count_ += joints.size() + keypoint_edges.size();
  // RCLCPP_INFO(this->get_logger(), "Added %ld markers", last_marker_count_);
}

void MasterNodeSingle::clear_markers() {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < last_marker_count_; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "joints";
    marker.id = static_cast<int>(i);
    marker.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
  last_marker_count_ = 0;
}

float MasterNodeSingle::computeWcj(float s_cj, Eigen::Vector3f joint_3d,
                                   Eigen::Matrix4f extrinsic_matrix) {
  Eigen::Matrix3f R = extrinsic_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f t = extrinsic_matrix.block<3, 1>(0, 3);

  Eigen::Vector3f camera_pos = -R.transpose() * t;

  float d_cj = (joint_3d - camera_pos).norm();

  Eigen::Vector3f dir_to_joint = joint_3d - camera_pos;
  Eigen::Vector3f camera_view_dir =
      -R.col(2); // Third column of the rotation matrix
  float cos_theta_cj = camera_view_dir.dot(dir_to_joint) /
                       (camera_view_dir.norm() * dir_to_joint.norm());

  float W_cj = s_cj * (1.0 / d_cj) * cos_theta_cj * cos_theta_cj;

  return W_cj;
}

Eigen::Vector3f
MasterNodeSingle::iterateWithWeights(const std::vector<float> &confidences) {

  Eigen::Vector3f X = A_.colPivHouseholderQr().solve(B_);

  for (int iteration = 0; iteration < max_iterations; iteration++) {
    std::vector<float> weights;
    for (size_t i = 0; i < confidences.size(); i++) {
      float s_cj = confidences[i];

      if (s_cj < 0.4) {
        s_cj *= s_cj;
      }

      float W_cj = computeWcj(
          s_cj, X, slaves_calibration_[camera_indices[i]].getExtrinsics());
      weights.push_back(W_cj);
    }

    Eigen::MatrixXf W(confidences.size(), confidences.size());
    for (size_t i = 0; i < weights.size(); i++) {
      W(i, i) = weights[i];
    }

    // A^T W A
    Eigen::MatrixXf AtWA = A_.transpose() * W * A_;

    // A^T W B
    Eigen::VectorXf AtWB = A_.transpose() * W * B_;

    Eigen::VectorXf X = AtWA.ldlt().solve(AtWB);

    /*
    //Old implementation, it is wrong...

    Eigen::MatrixXf A_weighted = A_;
    Eigen::VectorXf B_weighted = B_;

    for (size_t i = 0; i < weights.size(); i++) {
      A_weighted.row(2 * i) *= weights[i];
      A_weighted.row(2 * i + 1) *= weights[i];
      B_weighted.row(2 * i) *= weights[i];
      B_weighted.row(2 * i + 1) *= weights[i];
    }

    X = A_weighted.colPivHouseholderQr().solve(B_weighted);
    */
  }

  return X;
}

void MasterNodeSingle::filterFeedbacks() {
  rclcpp::Time current_time = this->get_clock()->now();
  for (size_t i = 0; i < slaves_feedback_.size(); i++) {
    rclcpp::Duration time_diff =
        current_time - slaves_feedback_[i].header.stamp;

    if (time_diff.seconds() <= MAX_TIME_DIFF) {
      filtered_feedbacks.push_back(slaves_feedback_[i]);
      camera_indices.push_back(i);
    }
  }
}

void MasterNodeSingle::scan_for_slaves() {
  auto topic_map = this->get_topic_names_and_types();

  for (const auto &topic_entry : topic_map) {
    const std::string &topic_name = topic_entry.first;
    std::smatch match;

    if (std::regex_match(topic_name, match, std::regex("(/slave_)(.*)")) &&
        subscribed_topics_names_.find(topic_name) ==
            subscribed_topics_names_.end()) {
      std::string node_name = match[2].str();
      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s",
                  topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "From node: %s", node_name.c_str());

      int index = subscribers_.size();
      slaves_feedback_.push_back(hpe_msgs::msg::Slave());

      auto sub = this->create_subscription<hpe_msgs::msg::Slave>(
          topic_name, 10,
          [this, index, topic_name](const hpe_msgs::msg::Slave::SharedPtr msg) {
            this->callback(*msg, index);
          });
      subscribers_.push_back(sub);
      subscribed_topics_names_.insert(topic_name);

      std::string calibration_service_name = "/calibration_" + node_name;
      requestCalibration(calibration_service_name);
      auto viz = std::make_shared<hpe_test::VisualizerNode>(node_name);
      visualizers.push_back(viz);
      executor_->add_node(viz);
    }
  }
}

void MasterNodeSingle::requestCalibration(std::string &service_name) {
  auto client = this->create_client<hpe_msgs::srv::Calibration>(service_name);

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Service %s is not available.",
                service_name.c_str());
    return;
  }

  auto request = std::make_shared<hpe_msgs::srv::Calibration::Request>();

  // Call the service
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Calibration received from %s.",
                service_name.c_str());
    slaves_calibration_.push_back(hpe_test::Calibration(result->calibration));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service %s.",
                 service_name.c_str());
  }
}

void MasterNodeSingle::buildAB_lists(int joint_n) {

  int num_rows = filtered_feedbacks.size() * 2;

  A_.resize(num_rows, 3);
  B_.resize(num_rows, 1);

  int row_offset = 0;

  for (size_t i = 0; i < filtered_feedbacks.size(); i++) {
    if ((int)filtered_feedbacks[0].all_joints[0].dim <= joint_n) {
      RCLCPP_ERROR(this->get_logger(), "ERROR, JOINT INDEX OUT OF BOUNDS");
    }

    float x = filtered_feedbacks[i].all_joints[0].x[joint_n];
    float y = filtered_feedbacks[i].all_joints[0].y[joint_n];

    Eigen::Vector2f p = Eigen::Vector2f::Zero();
    p(0) = x;
    p(1) = y;

    Eigen::Matrix<float, 2, 3> A_under =
        slaves_calibration_[camera_indices[i]].getARows(p);
    Eigen::Matrix<float, 2, 1> B_under =
        slaves_calibration_[camera_indices[i]].getBRows(p);

    A_.block(row_offset, 0, 2, 3) = A_under;
    B_.block(row_offset, 0, 2, 1) = B_under;

    row_offset += 2;
  }
}
}; // namespace hpe_test
