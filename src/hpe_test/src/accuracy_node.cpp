#include <hpe_test/accuracy_node.hpp>

using std::placeholders::_1;

namespace hpe_test {

// pairings[netw], metr
const std::vector<size_t> pairings = {1, 15, 17, 16, 18, 3,  9, 4, 10,
                                      5, 11, 6,  12, 7,  13, 8, 14};

double AccuracyNode::compute_distance(const geometry_msgs::msg::Point &netw,
                                      const std::string &netw_frame,
                                      const geometry_msgs::msg::Point &metr,
                                      const std::string &metr_frame) {
  double d = 0.0;

  geometry_msgs::msg::PointStamped netw_stamp;
  netw_stamp.point = netw;
  netw_stamp.header.frame_id = netw_frame;
  netw_stamp.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::PointStamped metr_stamp;
  metr_stamp.point = metr;
  metr_stamp.header.frame_id = metr_frame;
  metr_stamp.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::PointStamped netw_w =
      tf_buffer->transform(netw_stamp, "world");
  geometry_msgs::msg::PointStamped metr_w =
      tf_buffer->transform(metr_stamp, "world");

  d = sqrt(pow(netw_w.point.x - metr_w.point.x, 2) +
           pow(netw_w.point.y - metr_w.point.y, 2) +
           pow(netw_w.point.z - metr_w.point.z, 2));

  return d;
}

AccuracyNode::AccuracyNode() : rclcpp::Node("accuracy") {

  number_of_joints_tested = 0;
  max_pck = 0;
  pck = 0;
  number_of_joints_tested_high_conf = 0;
  max_pck_high_conf = 0;
  pck_high_conf = 0;
  mpjme = 0;
  hc_mpjme = 0;

  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  last_network = visualization_msgs::msg::MarkerArray();

  network_markers_subscription_ =
      this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/master_node_output", 10,
          std::bind(&AccuracyNode::network_callback, this, _1));

  metrabs_markers_subscription_ =
      this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/skeleton_markers", 10,
          std::bind(&AccuracyNode::metrabs_callback, this, _1));
}

AccuracyNode::~AccuracyNode() {

  double avg_pck = (double)pck / (double)number_of_joints_tested;
  double avg_hc_pck =
      (double)pck_high_conf / (double)number_of_joints_tested_high_conf;

  RCLCPP_WARN(
      this->get_logger(),
      "\n[Average pck (max): %f (%f)]\n[Average hc pck (max): %f (%f)]\n"
      "[MPJME (hcMPJME): %f (%f)]",
      avg_pck, max_pck, avg_hc_pck, max_pck_high_conf,
      mpjme / number_of_joints_tested,
      hc_mpjme / number_of_joints_tested_high_conf);
}

void AccuracyNode::metrabs_callback(
    const visualization_msgs::msg::MarkerArray &msg) {

  if (last_network == visualization_msgs::msg::MarkerArray())
    return;

  double iteration_pck = 0;
  double iteration_hc_pck = 0;

  size_t iteration_ck = 0;
  size_t iteration_hc_ck = 0;
  size_t iter_high_conf_n = 0;

  if (msg.markers.size() <= 0)
    return;

  if (msg.markers[0].points.size() <= 0)
    return;

  if (last_network.markers.size() < pairings.size())
    return;

  for (size_t i = 0; i < pairings.size(); i++) {
    double distance = compute_distance(last_network.markers[i].pose.position,
                                       last_network.markers[i].header.frame_id,
                                       msg.markers[0].points[pairings[i]],
                                       msg.markers[0].header.frame_id);

    mpjme += distance;

    RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);

    number_of_joints_tested++;
    // cheeky way to get confidence score :) since g = confidence
    if (last_network.markers[i].color.g > MIN_HIGH_CONF) {
      number_of_joints_tested_high_conf++;
      iter_high_conf_n++;
      hc_mpjme += distance;
      if (distance < DISTANCE_THRESHOLD) {
        pck_high_conf++;
        iteration_hc_ck++;
      }
    }

    if (distance < DISTANCE_THRESHOLD) {
      pck++;
      iteration_ck++;
    }
  }

  iteration_pck = (double)iteration_ck / (double)pairings.size();
  iteration_hc_pck = (double)iteration_hc_ck / (double)iter_high_conf_n;

  if (iteration_pck > max_pck)
    max_pck = iteration_pck;

  if (iteration_hc_pck > max_pck_high_conf)
    max_pck_high_conf = iteration_hc_pck;

  RCLCPP_INFO(this->get_logger(),
              "[Iteration pck (max): %f (%f)] [Iteration hc pck (max): %f "
              "(%f)] [MPJME (hcMPJME): %f (%f)]",
              iteration_pck, max_pck, iteration_hc_pck, max_pck_high_conf,
              mpjme / number_of_joints_tested,
              hc_mpjme / number_of_joints_tested_high_conf);
}

void AccuracyNode::network_callback(
    const visualization_msgs::msg::MarkerArray &msg) {
  this->last_network = msg;
}

} // namespace hpe_test
