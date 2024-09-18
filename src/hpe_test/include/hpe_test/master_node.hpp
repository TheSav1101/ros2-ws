#ifndef HPE_TEST__MASTER_NODE_HPP_
#define HPE_TEST__MASTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include "hpe_msgs/msg/detection.hpp"
#include "hpe_msgs/msg/box.hpp"
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_test/slave_node.hpp"
#include "hpe_test/data.hpp"


namespace hpe_test {

	class MasterNode : public rclcpp::Node {
		public:
			MasterNode(std::string name, int slave_n_);
			~MasterNode();

		private:
			//salva out di ogni raspberry
			void callback(const hpe_msgs::msg::Slave &msg);
			void loop();

			std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers_ = {};

			int slave_n = 0;
			std::vector<hpe_msgs::msg::Slave> slaves_feedback;
		};
}
#endif