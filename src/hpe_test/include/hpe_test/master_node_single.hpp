#ifndef HPE_TEST__MASTER_NODE_SINGLE_HPP_
#define HPE_TEST__MASTER_NODE_SINGLE_HPP_

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.hpp>
#include <hpe_msgs/msg/slave.hpp>
#include <hpe_msgs/srv/calibration.hpp>
#include <hpe_test/calibration.hpp>
#include <hpe_test/visualizer_node.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hpe_test {

class MasterNodeSingle : public rclcpp::Node {
public:
  MasterNodeSingle(const std::string name,
                   rclcpp::executors::MultiThreadedExecutor *executor);
  ~MasterNodeSingle();
  void shutdown();

private:
  // salva out di ogni raspberry
  void callback(const hpe_msgs::msg::Slave &msg, int index);
  void loop();
  void scan_for_slaves();
  void requestCalibration(const std::string &service_name);
  void filterFeedbacks();
  void triangulate_all();
  float computeWcj(const float &s_cj, const Eigen::Vector3f &joint_3d,
                   const Eigen::Matrix4f &extrinsic_matrix);

  Eigen::Vector3f triangulateWithoutWeights();
  Eigen::Vector3f iterateWithWeights(const std::vector<float> &confidences);
  void buildAB_lists(const int joint_n);
  void visualize_3d(std::vector<Eigen::Vector3f> &joints,
                    std::vector<float> &avg_conf);

  rclcpp::executors::MultiThreadedExecutor *executor_;
  std::vector<std::shared_ptr<hpe_test::VisualizerNode>> visualizers;

  // Linear algebra matrices
  Eigen::MatrixXf A_;
  Eigen::VectorXf B_;

  // Metrics
  float avg_delay;
  int delay_window;
  size_t last_marker_count_ = 0;

  const float MAX_TIME_DIFF = 5;
  const int max_iterations = 3;
  const int max_joints = 17;
  const float D_MIN = 1.2;
  const float D_MAX = 2.75;

  std::vector<hpe_msgs::msg::Slave> filtered_feedbacks;
  std::vector<int> camera_indices;

  std::vector<rclcpp::Subscription<hpe_msgs::msg::Slave>::SharedPtr>
      subscribers_ = {};
  std::vector<std::string> subscribed_topics_names_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  std::vector<hpe_msgs::msg::Slave> slaves_feedback_;

  std::vector<hpe_test::Calibration> slaves_calibration_;
  std::vector<rclcpp::Client<hpe_msgs::srv::Calibration>::SharedPtr>
      calibration_clients;

  rclcpp::TimerBase::SharedPtr scanner_;
  rclcpp::TimerBase::SharedPtr loop_;
};
} // namespace hpe_test
#endif
