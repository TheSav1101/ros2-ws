#ifndef HPE_TEST__SLAVE_NODE_HPP_
#define HPE_TEST__SLAVE_NODE_HPP_

#include "hpe_msgs/msg/box.hpp"
#include "hpe_msgs/msg/detection.hpp"
#include "hpe_msgs/msg/hpe2d.hpp"
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_msgs/srv/calibration.hpp"
#include "hpe_test/data.hpp"
#include "hpe_test/webcam_node.hpp"
#include "hpe_test/worker_node.hpp"
#include "tensorflow/lite/delegates/gpu/delegate.h"
#include <cv_bridge/cv_bridge.hpp>
#include <fstream>
#include <hpe_test/responses.hpp>
#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace hpe_test {

class SlaveNode : public rclcpp::Node {
public:
  SlaveNode(rclcpp::executors::MultiThreadedExecutor *executor,
            std::string name, std::string raw_topic, int model_,
            int detection_model_, int starting_workers,
            std::string calibration_topic);
  ~SlaveNode();
  void shutdown();

private:
  float computeIoU(const float *box1, const float *box2);
  std::vector<size_t> nonMaxSuppression(float *boxes, float *scores,
                                        size_t numBoxes, float iouThreshold,
                                        float minConfidence);
  void create_new_worker();
  void setupTensors();
  void saveCameraInfo(const sensor_msgs::msg::CameraInfo &msg);
  void callback(const sensor_msgs::msg::Image &msg);
  void compressedCallback(const sensor_msgs::msg::CompressedImage &msg);
  void open_camera();
  void calibrationService(
      const std::shared_ptr<hpe_msgs::srv::Calibration::Request> request
      [[maybe_unused]],
      std::shared_ptr<hpe_msgs::srv::Calibration::Response> response);
  void all_response_received_callback(Responses *response);

  rclcpp::executors::MultiThreadedExecutor *executor_;

  // model numbers (see data.cpp)
  int detection_model_n = 0;
  int hpe_model_n = 0;

  // Flag for calibration options
  std::atomic<bool> calibration_from_json;

  // transform stuff
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  sensor_msgs::msg::CameraInfo camera_info;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_boxes_;
  rclcpp::Service<hpe_msgs::srv::Calibration>::SharedPtr calibration_service_;
  rclcpp::Publisher<hpe_msgs::msg::Slave>::SharedPtr publisher_slave_;

  // clients list
  std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedPtr> clients_;

  // Read from some bags...
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_image_sub_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      subscription_info_;

  // tflite poiners
  std::unique_ptr<tflite::FlatBufferModel> model;
  std::unique_ptr<tflite::Interpreter> interpreter;

  // name
  std::string node_name;

  // AIUTO, il multithreading
  std::queue<std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture>>
      futures_vector_queue_;
  std::mutex queue_mutex_;
  std::atomic<bool> running_;
  std::vector<Responses *> responses_ptrs_;

  std::shared_ptr<hpe_test::WebcamNode> webcam_node;

  // worker stuff
  std::vector<std::shared_ptr<hpe_test::WorkerNode>> workers_;
  int workers_n = 0;

  // Metriche fps
  int delay_window;
  double avg_delay;
  double avg_delay_total;
  int delay_window_total;

  // TODO sort this stuff...
  int input_tensor_idx;
  int output_tensor_idx;
  TfLiteType inputType;
  TfLiteType outputType;
  int input_size;
  int output_size;
  TfLiteIntArray *input_dims;
  TfLiteIntArray *output_dims;
  std::string camera_frame;
  cv_bridge::CvImage cv_image_msg_bridge;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat small;
  sensor_msgs::msg::Image small_msg;
  float *input_data;
  float *output_data;
};

} // namespace hpe_test
#endif
