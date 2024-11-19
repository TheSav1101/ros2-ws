#ifndef HPE_TEST__LOOP_NODE_HPP_
#define HPE_TEST__LOOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_msgs/srv/estimate.hpp"

namespace hpe_test {

	class LoopNode : public rclcpp::Node {
		public:
			LoopNode(std::string name,
				std::queue<std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture>>* queue_ptr, 
				std::mutex* queue_mutex_ptr);
			~LoopNode();
			void shutdown();
			void loop();
			void start();

		private:
            //publisher
			rclcpp::Publisher<hpe_msgs::msg::Slave>::SharedPtr publisher_slave_;
					
			//name
			std::string node_name;

			//AIUTO, il multithreading
			std::queue<std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture>>* futures_vector_queue_ptr_;
			std::mutex* queue_mutex_ptr_;
            std::atomic<bool> running_;

			//Metriche fps
			int delay_window_loop;
            double avg_delay_loop;

			rclcpp::TimerBase::SharedPtr timer_;

		};

} 
#endif