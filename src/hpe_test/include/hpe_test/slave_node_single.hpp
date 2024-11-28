#ifndef HPE_TEST__SLAVE_NODE_SINGLE_HPP_
#define HPE_TEST__SLAVE_NODE_SINGLE_HPP_

#include <cv_bridge/cv_bridge.hpp>
#include <fstream>
#include <hpe_msgs/msg/box.hpp>
#include <hpe_msgs/msg/detection.hpp>
#include <hpe_msgs/msg/hpe2d.hpp>
#include <hpe_msgs/msg/slave.hpp>
#include <hpe_msgs/srv/calibration.hpp>
#include <hpe_test/data.hpp>
#include <hpe_test/webcam_node.hpp>
#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hpe_test {

class SlaveNodeSingle : public rclcpp::Node {
public:
  SlaveNodeSingle(const std::string name, const std::string raw_topic,
                  const int model_, const std::string calibration_topic);
  ~SlaveNodeSingle();
  void shutdown();

private:
  void setupTensors();
  void callback(const sensor_msgs::msg::Image &msg);
  void saveCameraInfo(const sensor_msgs::msg::CameraInfo &msg);
  void compressedCallback(const sensor_msgs::msg::CompressedImage &msg);
  void calibrationService(
      const std::shared_ptr<hpe_msgs::srv::Calibration::Request> request
      [[maybe_unused]],
      std::shared_ptr<hpe_msgs::srv::Calibration::Response> response);

  // model numbers (see data.cpp)
  int hpe_model_n = 0;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      publisher_boxes_;
  rclcpp::Publisher<hpe_msgs::msg::Slave>::SharedPtr publisher_slave_;
  rclcpp::Service<hpe_msgs::srv::Calibration>::SharedPtr calibration_service_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      subscription_info_;

  // Read from some bags...
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_image_sub_;

  // Flag for calibration options
  std::atomic<bool> calibration_from_json;

  sensor_msgs::msg::CameraInfo camera_info;

  // transform stuff
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // tflite poiners
  std::unique_ptr<tflite::FlatBufferModel> model;
  std::unique_ptr<tflite::Interpreter> interpreter;

  // name
  std::string node_name;

  // AIUTO, il multithreading
  std::thread webcam_thread;

  std::shared_ptr<hpe_test::WebcamNode> webcam_node;

  // Metriche fps
  int delay_window;
  double avg_delay;

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
  uint8_t *input_data;
  float *output_data;
};

} // namespace hpe_test
#endif
