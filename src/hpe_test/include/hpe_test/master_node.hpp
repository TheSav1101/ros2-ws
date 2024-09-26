#ifndef HPE_TEST__MASTER_NODE_HPP_
#define HPE_TEST__MASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <regex>
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_msgs/srv/calibration.hpp"


namespace hpe_test {

	class MasterNode : public rclcpp::Node {
		public:
			MasterNode(std::string name);
			~MasterNode();

		private:
			//salva out di ogni raspberry
			void callback(const hpe_msgs::msg::Slave &msg, int index, std::string topic);
			void loop();
			void scan_for_slaves();
			void requestCalibration(std::string &service_name);
			void filterFeedbacks(std::vector<hpe_msgs::msg::Slave> &filtered_feedbacks, std::vector<int> &camera_indices);

			const float MAX_TIME_DIFF = 0.5;
			std::vector<rclcpp::Subscription<hpe_msgs::msg::Slave>::SharedPtr> subscribers_ = {};
			std::set<std::string> subscribed_topics_names_;

			std::vector<hpe_msgs::msg::Slave> slaves_feedback_;

			std::vector<hpe_msgs::msg::Calibration> slaves_calibration_;

		    rclcpp::TimerBase::SharedPtr scanner_;

		};
}
#endif