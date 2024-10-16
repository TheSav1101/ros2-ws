#ifndef HPE_TEST__SLAVE_NODE_HPP_
#define HPE_TEST__SLAVE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <vector>
#include "hpe_msgs/msg/hpe2d.hpp"
#include "hpe_msgs/msg/detection.hpp"
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_msgs/msg/box.hpp"
#include "hpe_test/worker_node.hpp"
#include "hpe_test/webcam_node.hpp"
#include "hpe_test/data.hpp"
#include "hpe_msgs/srv/calibration.hpp"

namespace hpe_test {

	class SlaveNode : public rclcpp::Node {
		public:
			SlaveNode(std::string name, std::string raw_topic, int model_, int detection_model_, int starting_workers);
			~SlaveNode();
			void shutdown();


		private:
			float computeIoU(const float* box1, const float* box2);
			std::vector<size_t> nonMaxSuppression(float* boxes, float* scores, size_t numBoxes, float iouThreshold, float minConfidence);
			void create_new_worker();
			void setupTensors();
			void callback(const sensor_msgs::msg::Image &msg);
			void open_camera();
			void findPpl(cv::Mat&, std_msgs::msg::Header);
			void calibrationService(const std::shared_ptr<hpe_msgs::srv::Calibration::Request> request, std::shared_ptr<hpe_msgs::srv::Calibration::Response> response);
			void loop();

			//model numbers (see data.cpp)
			int detection_model_n = 0; 
			int hpe_model_n = 0;

			//publishers
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_boxes_;
			rclcpp::Publisher<hpe_msgs::msg::Slave>::SharedPtr publisher_slave_;
			std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedPtr> clients_;
			rclcpp::Service<hpe_msgs::srv::Calibration>::SharedPtr calibration_service_;
			
			//subscriber
			rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

			//tflite poiners
			std::unique_ptr<tflite::FlatBufferModel> model;
			std::unique_ptr<tflite::Interpreter> interpreter;

					
			//name
			std::string node_name;

			//AIUTO, il multithreading
			std::thread webcam_thread;
			std::thread loop_thread;
			std::queue<std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture>> futures_vector_queue_;
			std::mutex queue_mutex_;
			std::atomic<bool> running_;

			std::shared_ptr<hpe_test::WebcamNode> webcam_node;

			//worker stuff
			std::vector<std::thread> worker_threads;
			int workers_n = 0;

			//Metriche fps
			int delay_window;
            double avg_delay;
			int delay_window_loop;
            double avg_delay_loop;

			//TODO sort this stuff...
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

} 
#endif