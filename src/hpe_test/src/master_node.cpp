#include <hpe_test/master_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hpe_test{

    MasterNode::MasterNode(std::string name) : Node(name){
        subscribers_ = {};
        slaves_feedback_ = {};
        scan_for_slaves();
        scanner_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MasterNode::scan_for_slaves, this));

        loop();
    }

    MasterNode::~MasterNode(){
    }

    void MasterNode::callback(const hpe_msgs::msg::Slave &msg, int index, std::string topic){
        slaves_feedback_[index] = msg;
        RCLCPP_INFO(this->get_logger(), "Recived from %s", topic.c_str());

    }

    void MasterNode::loop(){
        while(rclcpp::ok()){
            
            //Filter feedbacks by time
            std::vector<hpe_msgs::msg::Slave> filtered_feedbacks = {};
            std::vector<int> camera_indices = {};
            filterFeedbacks(filtered_feedbacks, camera_indices);
            
            
        }
    }

    void MasterNode::filterFeedbacks(std::vector<hpe_msgs::msg::Slave> &filtered_feedbacks, std::vector<int> &camera_indices){
        rclcpp::Time current_time = this->get_clock()->now();
        for (size_t i = 0; i < slaves_feedback_.size(); ++i)
        {
            rclcpp::Duration time_diff = current_time - slaves_feedback_[i].header.stamp;

            if (time_diff.seconds() <= MAX_TIME_DIFF)
            {
                filtered_feedbacks.push_back(slaves_feedback_[i]);
                camera_indices.push_back(i);
            }
        }
    }

    void MasterNode::scan_for_slaves()
    {
        auto topic_map = this->get_topic_names_and_types();

        for (const auto &topic_entry : topic_map)
        {
            const std::string &topic_name = topic_entry.first;
            std::smatch match;

            if (std::regex_match(topic_name, match, std::regex("(/slave_)(.*)") ) && subscribed_topics_names_.find(topic_name) == subscribed_topics_names_.end())
            {
                std::string node_name = match[2].str();
                RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_name.c_str());
                RCLCPP_INFO(this->get_logger(), "From node: %s", node_name.c_str());

                int index = subscribers_.size();

                auto sub = this->create_subscription<hpe_msgs::msg::Slave>(
                topic_name, 10, [this, index, topic_name](const hpe_msgs::msg::Slave::SharedPtr msg) {
                    this->callback(*msg, index, topic_name);
                });
                subscribers_.push_back(sub);
                subscribed_topics_names_.insert(topic_name);
                slaves_feedback_.push_back(hpe_msgs::msg::Slave());

                std::string calibration_service_name = "/calibration_" + node_name;
                requestCalibration(calibration_service_name);
                
            }
        }
    }

    void MasterNode::requestCalibration(std::string &service_name){
        auto client = this->create_client<hpe_msgs::srv::Calibration>(service_name);
        
        if (!client->wait_for_service(std::chrono::seconds(2))){
            RCLCPP_WARN(this->get_logger(), "Service %s is not available.", service_name.c_str());
            return;
        }

        auto request = std::make_shared<hpe_msgs::srv::Calibration::Request>();
        
        // Call the service
        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Calibration received from %s.", service_name.c_str());
            slaves_calibration_.push_back(result->calibration);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service %s.", service_name.c_str());
        }
    }


};