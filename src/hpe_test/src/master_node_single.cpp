#include <hpe_test/master_node_single.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hpe_test{

    MasterNodeSingle::MasterNodeSingle(std::string name) : Node(name){
        avg_delay = 0.0;
		delay_window = 0;

        filtered_feedbacks = {};
        camera_indices = {};

        subscribers_ = {};
        slaves_feedback_ = {};
        scan_for_slaves();
        scanner_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&MasterNodeSingle::scan_for_slaves, this));

        loop_thread = std::thread(&MasterNodeSingle::loop, this);
		loop_thread.detach();
    }

    void MasterNodeSingle::shutdown(){
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
		running_ = false;

		if(loop_thread.joinable()){
			loop_thread.join();
			RCLCPP_INFO(this->get_logger(), "Loop thread joined...");
		}
        RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay);
    }

    MasterNodeSingle::~MasterNodeSingle(){
        running_ = false;
        if(loop_thread.joinable()){
			loop_thread.join();
			RCLCPP_INFO(this->get_logger(), "Loop thread joined...");
		}
    }

    void MasterNodeSingle::callback(const hpe_msgs::msg::Slave &msg, int index, std::string topic){
        slaves_feedback_[index] = msg;
        RCLCPP_INFO(this->get_logger(), "Recived from %s", topic.c_str());
    }

    void MasterNodeSingle::loop(){
        running_ = true;
        rclcpp::Rate loop_rate(30);
        while(running_){
            auto start = this->get_clock()->now();
            
            //Filter feedbacks by time
            filtered_feedbacks = {};
            camera_indices = {};
            filterFeedbacks();

            triangulate_all(); 
            
			double delay = (this->get_clock()->now() - start).seconds();
			avg_delay = (avg_delay*delay_window + delay);
			delay_window++;
			avg_delay /= delay_window;
            loop_rate.sleep();
        }
    }

    void MasterNodeSingle::triangulate_all(){



    }

    Eigen::Vector3f MasterNodeSingle::triangulateWithoutWeights(const std::vector<Eigen::Matrix<float, 2, 3>>& A_list, const std::vector<Eigen::Matrix<float, 2, 1>>& B_list){
        Eigen::MatrixXf A_combined;
        Eigen::VectorXf B_combined;
 
        for (size_t i = 0; i < A_list.size(); ++i) {
            A_combined.conservativeResize(A_combined.rows() + 2, 3);
            A_combined.bottomRows(2) = A_list[i];

            B_combined.conservativeResize(B_combined.rows() + 2, 1);
            B_combined.bottomRows(2) = B_list[i];
        }

        Eigen::Vector3f X = A_combined.colPivHouseholderQr().solve(B_combined);

        return X;
    }

    float MasterNodeSingle::computeWcj(float s_cj, Eigen::Vector3f joint_3d, Eigen::Matrix4f extrinsic_matrix) {
        Eigen::Matrix3f R = extrinsic_matrix.block<3, 3>(0, 0);
        Eigen::Vector3f t = extrinsic_matrix.block<3, 1>(0, 3);

        Eigen::Vector3f camera_pos = -R.transpose() * t;

        float d_cj = (joint_3d - camera_pos).norm();

        Eigen::Vector3f dir_to_joint = joint_3d - camera_pos;
        Eigen::Vector3f camera_view_dir = -R.col(2); // Third column of the rotation matrix
        float cos_theta_cj = camera_view_dir.dot(dir_to_joint) / (camera_view_dir.norm() * dir_to_joint.norm());

        float W_cj = s_cj * (1.0 / d_cj) * cos_theta_cj * cos_theta_cj;

        return W_cj;
    }

    Eigen::Vector3f MasterNodeSingle::iterateWithWeights(std::vector<Eigen::Matrix<float, 2, 3>>& A_list, std::vector<Eigen::Matrix<float, 2, 1>>& B_list, const std::vector<float>& confidences) {
    
        Eigen::Vector3f X = triangulateWithoutWeights(A_list, B_list);
    
        for (int iteration = 0; iteration < max_iterations; iteration++) {
            std::vector<float> weights;
            for (size_t i = 0; i < A_list.size(); i++) {
                float s_cj = confidences[i];
                float W_cj = computeWcj(s_cj, X, slaves_calibration_[camera_indices[i]].getExtrinsics());
                weights.push_back(W_cj);
            }

            Eigen::MatrixXf A_combined;
            Eigen::VectorXf B_combined;
            for (size_t i = 0; i < A_list.size(); ++i) {
                Eigen::Matrix<float, 2, 3> A_weighted = weights[i] * A_list[i];
                Eigen::Matrix<float, 2, 1> B_weighted = weights[i] * B_list[i];

                A_combined.conservativeResize(A_combined.rows() + 2, 3);
                A_combined.bottomRows(2) = A_weighted;

                B_combined.conservativeResize(B_combined.rows() + 2, 1);
                B_combined.bottomRows(2) = B_weighted;
            }

            X = A_combined.colPivHouseholderQr().solve(B_combined);
        }
        
        return X;
    }

    void MasterNodeSingle::filterFeedbacks(){
        rclcpp::Time current_time = this->get_clock()->now();
        for (size_t i = 0; i < slaves_feedback_.size(); i++)
        {
            rclcpp::Duration time_diff = current_time - slaves_feedback_[i].header.stamp;

            if (time_diff.seconds() <= MAX_TIME_DIFF)
            {
                filtered_feedbacks.push_back(slaves_feedback_[i]);
                camera_indices.push_back(i);
            }
        }
    }

    void MasterNodeSingle::scan_for_slaves(){
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

    void MasterNodeSingle::requestCalibration(std::string &service_name){
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
            slaves_calibration_.push_back(hpe_test::Calibration(result->calibration));
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service %s.", service_name.c_str());
        }
    }

    std::vector<Eigen::Matrix<float, 2, 3>> MasterNodeSingle::buildA_list(int joint_n, std::vector<int> cameras, std::vector<int> skeletons){
        std::vector<Eigen::Matrix<float, 2, 3>> out = {};
        
        for(size_t i = 0; i < cameras.size(); i++){

            if((int) filtered_feedbacks.size() <= cameras[i])
                RCLCPP_ERROR(this->get_logger(), "ERROR, CAMERA INDEX OUT OF BOUNDS");
            if((int) filtered_feedbacks[cameras[i]].all_joints.size() <= skeletons[i])
                RCLCPP_ERROR(this->get_logger(), "ERROR, SKELETON INDEX OUT OF BOUNDS");
            if((int) filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].x.size() <= joint_n)
                RCLCPP_ERROR(this->get_logger(), "ERROR, JOINT INDEX OUT OF BOUNDS");


            float x = filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].x[joint_n];
            float y = filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].y[joint_n];

            Eigen::Vector2f p = Eigen::Vector2f::Zero();
            p(0) = x;
            p(1) = y;

            out.push_back(slaves_calibration_[camera_indices[cameras[i]]].getARows(p));
        }

        return out;
    }
    std::vector<Eigen::Matrix<float, 2, 1>> MasterNodeSingle::buildB_list(int joint_n, std::vector<int> cameras, std::vector<int> skeletons){
        std::vector<Eigen::Matrix<float, 2, 1>> out = {};

        for(size_t i = 0; i < cameras.size(); i++){

            if((int) filtered_feedbacks.size() <= cameras[i])
                RCLCPP_ERROR(this->get_logger(), "ERROR, CAMERA INDEX OUT OF BOUNDS");
            if((int) filtered_feedbacks[cameras[i]].all_joints.size() <= skeletons[i])
                RCLCPP_ERROR(this->get_logger(), "ERROR, SKELETON INDEX OUT OF BOUNDS");
            if((int) filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].x.size() <= joint_n)
                RCLCPP_ERROR(this->get_logger(), "ERROR, JOINT INDEX OUT OF BOUNDS");


            float x = filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].x[joint_n];
            float y = filtered_feedbacks[cameras[i]].all_joints[skeletons[i]].y[joint_n];

            Eigen::Vector2f p = Eigen::Vector2f::Zero();
            p(0) = x;
            p(1) = y;

            out.push_back(slaves_calibration_[camera_indices[cameras[i]]].getBRows(p));
        }
        
        return out;
    }


};