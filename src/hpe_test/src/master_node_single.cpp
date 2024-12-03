#include <hpe_test/master_node_single.hpp>

namespace hpe_test {

MasterNodeSingle::MasterNodeSingle(
    const std::string name, rclcpp::executors::MultiThreadedExecutor *executor)
    : Node(name) {
  avg_delay = 0.0;
  delay_window = 0;
  last_marker_count_ = 0;

  executor_ = executor;

  filtered_feedbacks = {};
  camera_indices = {};

  subscribers_ = {};
  slaves_feedback_ = {};

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "master_node_output", rclcpp::QoS(10));

  scan_for_slaves();

  scanner_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MasterNodeSingle::scan_for_slaves, this));

  loop_ = this->create_wall_timer(std::chrono::milliseconds(15),
                                  std::bind(&MasterNodeSingle::loop, this));
}

void MasterNodeSingle::shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay);
}

MasterNodeSingle::~MasterNodeSingle() {
  RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay);
}

void MasterNodeSingle::callback(const hpe_msgs::msg::Slave &msg, int index) {
  slaves_feedback_[index] = msg;
  // RCLCPP_INFO(this->get_logger(), "Recived from %d", index);
}

void MasterNodeSingle::loop() {
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
}

void MasterNodeSingle::triangulate_all() {
  if (filtered_feedbacks.size() <= 1) {
    RCLCPP_WARN(this->get_logger(), "Too few responses: %ld",
                filtered_feedbacks.size());
    return;
  }
  std::string s = "";

  for (size_t i : camera_indices) {
    s += " " + subscribed_topics_names_[i];
  }

  RCLCPP_INFO(this->get_logger(), "Using responses from %s", s.c_str());

  std::vector<Eigen::Vector3f> joints3d = {};
  std::vector<float> avg_conf = {};
  float very_avg_conf = 0.0;
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

    very_avg_conf += avg_conf_;
  }
  very_avg_conf /= max_joints;
  if (very_avg_conf >= 0.2)
    visualize_3d(joints3d, avg_conf);
  else
    RCLCPP_WARN(this->get_logger(), "Confidence too low to publish markers: %f",
                very_avg_conf);
}

void MasterNodeSingle::visualize_3d(std::vector<Eigen::Vector3f> &joints,
                                    std::vector<float> &avg_conf) {
  visualization_msgs::msg::MarkerArray marker_array;

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
  }

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
    edge_marker.scale.x = 0.02;

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
}

float MasterNodeSingle::computeWcj(const float &s_cj,
                                   const Eigen::Vector3f &joint_3d,
                                   const Eigen::Matrix4f &extrinsic_matrix) {
  Eigen::Matrix3f R = extrinsic_matrix.block<3, 3>(0, 0);
  Eigen::Vector3f t = extrinsic_matrix.block<3, 1>(0, 3);

  Eigen::Vector3f camera_pos = -R.transpose() * t;

  float d_cj = (joint_3d - camera_pos).norm();

  if (0 <= d_cj && d_cj < D_MIN) {
    d_cj /= D_MIN;
  } else if (D_MIN <= d_cj && d_cj < D_MAX) {
    d_cj = 1.0;
  } else if (D_MAX <= d_cj && d_cj < 10) {
    float a = 10 - D_MAX;
    d_cj -= D_MAX;
    d_cj /= a;
  } else {
    d_cj = 0;
  }

  float W_cj = s_cj * d_cj;

  if (W_cj <= 0.00000000000000000000000000001) {
    W_cj = 0.00000000000000000000000000001;
  }

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

    Eigen::MatrixXf W(confidences.size() * 2, confidences.size() * 2);
    for (size_t i = 0; i < 2 * weights.size(); i += 2) {
      W(i, i) = weights[i];
      W(i + 1, i + 1) = weights[i];
    }

    // A^T W A
    Eigen::MatrixXf AtWA = A_.transpose() * W * A_;

    // A^T W B
    Eigen::VectorXf AtWB = A_.transpose() * W * B_;

    Eigen::VectorXf X = AtWA.ldlt().solve(AtWB);
  }

  return X;
}

void MasterNodeSingle::filterFeedbacks() {
  rclcpp::Time current_time = this->get_clock()->now();
  for (size_t i = 0; i < slaves_feedback_.size(); i++) {
    rclcpp::Duration time_diff =
        current_time - slaves_feedback_[i].header.stamp;

    if (!slaves_feedback_[i].all_joints.size())
      continue;

    if (time_diff.seconds() <= MAX_TIME_DIFF &&
        slaves_calibration_[i].isReady()) {
      filtered_feedbacks.push_back(slaves_feedback_[i]);
      camera_indices.push_back(i);
    } else {
      rclcpp::Duration time_diff2 =
          (rclcpp::Time)slaves_feedback_[i].header.stamp - current_time;
      if (time_diff2.seconds() <= MAX_TIME_DIFF &&
          slaves_calibration_[i].isReady()) {
        filtered_feedbacks.push_back(slaves_feedback_[i]);
        camera_indices.push_back(i);
      } else {
        rclcpp::Duration time_diff3 = time_diff2;
        if (time_diff < time_diff2)
          time_diff3 = time_diff;

        RCLCPP_WARN(this->get_logger(), "time diff for %s is %f",
                    subscribed_topics_names_[i].c_str(), time_diff3.seconds());
      }
    }
  }
}

void MasterNodeSingle::scan_for_slaves() {
  auto topic_map = this->get_topic_names_and_types();

  for (const auto &topic_entry : topic_map) {
    const std::string &topic_name = topic_entry.first;
    std::smatch match;

    if (std::regex_match(topic_name, match, std::regex("(/slave_)(.*)")) &&
        std::find(subscribed_topics_names_.begin(),
                  subscribed_topics_names_.end(),
                  topic_name) == subscribed_topics_names_.end()) {
      std::string node_name = match[2].str();
      RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s",
                  topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "From node: %s", node_name.c_str());

      int index = subscribers_.size();
      slaves_feedback_.push_back(hpe_msgs::msg::Slave());

      auto sub = this->create_subscription<hpe_msgs::msg::Slave>(
          topic_name, rclcpp::QoS(2),
          [this, index, topic_name](const hpe_msgs::msg::Slave::SharedPtr msg) {
            this->callback(*msg, index);
          });
      subscribers_.push_back(sub);
      subscribed_topics_names_.push_back(topic_name);

      std::string calibration_service_name = "/calibration_" + node_name;
      requestCalibration(calibration_service_name);
      auto viz = std::make_shared<hpe_test::VisualizerNode>(node_name);
      visualizers.push_back(viz);
      executor_->add_node(viz);
    }
  }
}

void MasterNodeSingle::requestCalibration(const std::string &service_name) {

  size_t index = calibration_clients.size();
  calibration_clients.push_back(
      this->create_client<hpe_msgs::srv::Calibration>(service_name));

  if (!calibration_clients[index]->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "Service %s is not available.",
                service_name.c_str());
    return;
  }

  auto request = std::make_shared<hpe_msgs::srv::Calibration::Request>();
  size_t i = slaves_calibration_.size();
  slaves_calibration_.push_back(hpe_test::Calibration());

  auto response_received_callback =
      [this, i, service_name](
          rclcpp::Client<hpe_msgs::srv::Calibration>::SharedFuture future) {
        auto result = future.get();
        slaves_calibration_[i] = hpe_test::Calibration(result->calibration);
        RCLCPP_INFO(this->get_logger(), "Calibration received from %s.",
                    service_name.c_str());
      };

  auto future = calibration_clients[index]->async_send_request(
      request, response_received_callback);

  RCLCPP_INFO(this->get_logger(), "Calibration requested to %s.",
              service_name.c_str());
}

void MasterNodeSingle::buildAB_lists(const int joint_n) {

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
