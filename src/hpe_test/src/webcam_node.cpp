#include <hpe_test/webcam_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


namespace hpe_test {
    WebcamNode::WebcamNode(std::string name) : Node("webcam_" + name){
        node_name = "webcam_" + name;
        publisher_raw_ = this->create_publisher<sensor_msgs::msg::Image>("/webcam_" + name, 10);
        open_camera();
    }

	WebcamNode::~WebcamNode(){
		
	}

    void WebcamNode::open_camera() {
		cap = cv::VideoCapture(0);

		RCLCPP_INFO(this->get_logger(), "Ready");
		if (!cap.isOpened()) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: no video feed...");
		}

		RCLCPP_INFO(this->get_logger(), "Starting loop");
		
		while (rclcpp::ok()) {
			cv::Mat frame;
			cap >> frame;

			if (frame.empty()) {
				RCLCPP_ERROR(this->get_logger(), "ERROR: missing a frame...");
			}

			std_msgs::msg::Header header;
			header.set__stamp(this->get_clock()->now());
			header.set__frame_id(node_name);

            cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
            cv_image_msg_bridge.toImageMsg(msg);
            publisher_raw_->publish(msg);
		}

		cap.release();
	}
}