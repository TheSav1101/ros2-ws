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
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace hpe_test {

	class MasterNodeSingle : public rclcpp::Node {
		public:
			MasterNodeSingle(std::string name);
			~MasterNodeSingle();
			void shutdown();

		private:
			//salva out di ogni raspberry
			void callback(const hpe_msgs::msg::Slave &msg, int index);
			void loop();
			void scan_for_slaves();
			void requestCalibration(std::string &service_name);
			void filterFeedbacks();
			void triangulate_all();
			float computeWcj(float s_cj, Eigen::Vector3f joint_3d, Eigen::Matrix4f extrinsic_matrix);
			
			Eigen::Vector3f triangulateWithoutWeights();
			Eigen::Vector3f iterateWithWeights(const std::vector<float>& confidences);
			void buildAB_lists(int joint_n);
			void visualize_3d(std::vector<Eigen::Vector3f> &joints, std::vector<float>& avg_conf);
			void clear_markers();

			//Linear algebra matrices
			Eigen::MatrixXf A_;
        	Eigen::VectorXf B_;

			//Metrics
			float avg_delay;
			int delay_window;
			size_t last_marker_count_ = 0;

			//Loop stuff
			std::atomic<bool> running_;
			std::thread loop_thread;

			const float MAX_TIME_DIFF = 1;
			const int max_iterations = 3;
			const int max_joints = 17;

			std::vector<hpe_msgs::msg::Slave> filtered_feedbacks;
            std::vector<int> camera_indices;

			std::vector<rclcpp::Subscription<hpe_msgs::msg::Slave>::SharedPtr> subscribers_ = {};
			std::set<std::string> subscribed_topics_names_;
			rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

			std::vector<hpe_msgs::msg::Slave> slaves_feedback_;

			std::vector<hpe_test::Calibration> slaves_calibration_;

		    rclcpp::TimerBase::SharedPtr scanner_;

		};
}
#endif