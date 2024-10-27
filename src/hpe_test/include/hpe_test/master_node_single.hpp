#ifndef HPE_TEST__MASTER_NODE_SINGLE_HPP_
#define HPE_TEST__MASTER_NODE_SINGLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <regex>
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_msgs/srv/calibration.hpp"
#include <hpe_test/calibration.hpp>


namespace hpe_test {

	class MasterNodeSingle : public rclcpp::Node {
		public:
			MasterNodeSingle(std::string name);
			~MasterNodeSingle();
			void shutdown();

		private:
			//salva out di ogni raspberry
			void callback(const hpe_msgs::msg::Slave &msg, int index, std::string topic);
			void loop();
			void scan_for_slaves();
			void requestCalibration(std::string &service_name);
			void filterFeedbacks();
			void triangulate_all();
			float computeWcj(float s_cj, Eigen::Vector3f joint_3d, Eigen::Matrix4f extrinsic_matrix);
			
			Eigen::Vector3f triangulateWithoutWeights(const std::vector<Eigen::Matrix<float, 2, 3>>& A_list, const std::vector<Eigen::Matrix<float, 2, 1>>& B_list);
			Eigen::Vector3f iterateWithWeights(std::vector<Eigen::Matrix<float, 2, 3>>& A_list, std::vector<Eigen::Matrix<float, 2, 1>>& B_list, const std::vector<float>& confidences);
			std::vector<Eigen::Matrix<float, 2, 3>> buildA_list(int joint_n, std::vector<int> cameras, std::vector<int> skeletons);
			std::vector<Eigen::Matrix<float, 2, 1>> buildB_list(int joint_n, std::vector<int> cameras, std::vector<int> skeletons);
			//Metrics
			float avg_delay;
			int delay_window;

			//Loop stuff
			std::atomic<bool> running_;
			std::thread loop_thread;

			const float MAX_TIME_DIFF = 0.15;
			const int max_iterations = 3;

			std::vector<hpe_msgs::msg::Slave> filtered_feedbacks;
            std::vector<int> camera_indices;

			std::vector<rclcpp::Subscription<hpe_msgs::msg::Slave>::SharedPtr> subscribers_ = {};
			std::set<std::string> subscribed_topics_names_;

			std::vector<hpe_msgs::msg::Slave> slaves_feedback_;

			std::vector<hpe_test::Calibration> slaves_calibration_;

		    rclcpp::TimerBase::SharedPtr scanner_;

		};
}
#endif