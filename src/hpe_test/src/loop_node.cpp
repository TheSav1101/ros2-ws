#include <hpe_test/loop_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


namespace hpe_test{

    LoopNode::LoopNode(std::string name, 
    std::queue<std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture>>* queue_ptr, 
    std::mutex* queue_mutex_ptr) : Node("loop_" + name){

        futures_vector_queue_ptr_ = queue_ptr;
        queue_mutex_ptr_ = queue_mutex_ptr;
        running_ = true;

        delay_window_loop = 0;
        avg_delay_loop = 0;

		node_name = name;

        publisher_slave_ = this->create_publisher<hpe_msgs::msg::Slave>("/slave_" + name, 10);
    }

	void LoopNode::start(){
		RCLCPP_INFO(this->get_logger(), "Creating timer...");
		timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&LoopNode::loop, this));
	}

    LoopNode::~LoopNode(){
        running_ = false;
        RCLCPP_WARN(this->get_logger(), "Average FPS: %f, total frames: %d", 1 / avg_delay_loop, delay_window_loop);
    }

    void LoopNode::shutdown(){
        running_ = false;
		timer_.reset();
    }
    
    void LoopNode::loop(){
		std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture> current_futures = {};
		RCLCPP_WARN(this->get_logger(), "Start");

		auto start = this->get_clock()->now();
		{
			std::lock_guard<std::mutex> lock(*queue_mutex_ptr_);

			if (futures_vector_queue_ptr_->empty()) {
				return;
			}
			current_futures = futures_vector_queue_ptr_->front();
			futures_vector_queue_ptr_->pop();
			if(current_futures.empty()){
				return;
			}	
		}

		std_msgs::msg::Header header = std_msgs::msg::Header();
		std::vector<hpe_msgs::msg::Joints2d> all_joints = {};

		for (size_t i = 0; i < current_futures.size(); i++){
			auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), current_futures[i], std::chrono::milliseconds(100));
			if(result == rclcpp::FutureReturnCode::SUCCESS){
				try{
					auto response = current_futures[i].get();
					all_joints.push_back(response->hpe2d.joints);
					header = response->hpe2d.header;
					RCLCPP_WARN(this->get_logger(), "GOT ONE");
				}
				catch(const std::exception& e){
					RCLCPP_ERROR(this->get_logger(), "Failed to get response from worker_%s_%zu", node_name.c_str(), i);
				}
			}else {
				RCLCPP_ERROR(this->get_logger(), "Bad future from worker_%s_%zu", node_name.c_str(), i);
			}
		}

		hpe_msgs::msg::Slave slave_msg = hpe_msgs::msg::Slave();
		slave_msg.header = header;
		slave_msg.all_joints = all_joints;
		slave_msg.skeletons_n = all_joints.size();

		publisher_slave_->publish(slave_msg);
		double delay = (this->get_clock()->now() - start).seconds();
		avg_delay_loop = (avg_delay_loop*delay_window_loop + delay);
		delay_window_loop++;
		avg_delay_loop /= delay_window_loop;
	}

    
}