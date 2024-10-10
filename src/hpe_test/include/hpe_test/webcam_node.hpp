#ifndef HPE_TEST__WEBCAM_NODE_HPP_
#define HPE_TEST__WEBCAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>

namespace hpe_test {

	class WebcamNode : public rclcpp::Node {
		public:
			WebcamNode(std::string name);
			~WebcamNode();
			void stop();

		private:
			void open_camera();

			cv_bridge::CvImage cv_image_msg_bridge;
			cv::VideoCapture cap;
			sensor_msgs::msg::Image msg;
			std::atomic<bool> running_;
			//publishers
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_raw_;
					
			//name
			std::string node_name;
		};

} 
#endif