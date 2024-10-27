#include <hpe_test/slave_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;


namespace hpe_test {

	float SlaveNode::computeIoU(const float* box1, const float* box2) {
		float half_w1 = box1[2] / 2.0;
		float half_h1 = box1[3] / 2.0;
		float half_w2 = box2[2] / 2.0;
		float half_h2 = box2[3] / 2.0;

		float x1 = std::max(box1[0] - half_w1, box2[0] - half_w2);
		float y1 = std::max(box1[1] - half_h1, box2[1] - half_h2);
		float x2 = std::min(box1[0] + half_w1, box2[0] + half_w2);
		float y2 = std::min(box1[1] + half_h1, box2[1] + half_h2);

		float intersectionWidth = std::max(0.0f, x2 - x1);
		float intersectionHeight = std::max(0.0f, y2 - y1);
		float intersectionArea = intersectionWidth * intersectionHeight;

		float box1Area = box1[2] * box1[3];
		float box2Area = box2[2] * box2[3];

		float unionArea = box1Area + box2Area - intersectionArea;
		if (unionArea <= 0.0) {
			return 0.0;
		}

		return intersectionArea / unionArea;
	}

	std::vector<size_t> SlaveNode::nonMaxSuppression(float* boxes, float* scores, size_t numBoxes, float iouThreshold, float minConfidence) {
		std::vector<size_t> indices(numBoxes);
		std::vector<size_t> final_indices;

		std::iota(indices.begin(), indices.end(), 0);

		std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
			return scores[i1] > scores[i2];
		});

		std::vector<bool> suppressed(numBoxes, false);

		for (size_t i = 0; i < numBoxes; ++i) {
			size_t idx = indices[i];
			if (scores[idx] < minConfidence || suppressed[idx]) continue;
			final_indices.push_back(idx);

			for (size_t j = i + 1; j < numBoxes; ++j) {
				size_t compareIdx = indices[j];
				if (computeIoU(boxes + 4 * idx, boxes + 4 * compareIdx) > iouThreshold) {
					suppressed[compareIdx] = true;
				}
			}
		}
		return final_indices;
	}

	void SlaveNode::create_new_worker(){

		std::string name = node_name + "_" + std::to_string(workers_n);
		workers_n++;

		clients_.push_back(this->create_client<hpe_msgs::srv::Estimate>("estimate" + name));
		
		worker_threads.push_back(std::thread([name, this]() {
			auto worker_node = std::make_shared<hpe_test::WorkerNode>(name, hpe_model_n);
			rclcpp::spin(worker_node);
		}));
	}

  	void SlaveNode::findPpl(cv::Mat &img, std_msgs::msg::Header header) {
		auto start = this->get_clock()->now();
		cv::resize(img, small, cv::Size(DETECTION_MODEL_WIDTH[detection_model_n], DETECTION_MODEL_HEIGHT[detection_model_n]), cv::INTER_LINEAR);

		// TODO togliere sta roba
		cv::Mat floatImg;
		small.convertTo(floatImg, CV_32FC3, 1.0/255.0);
	
		memcpy(input_data, floatImg.data, input_size * sizeof(float));

		if (!interpreter) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: Interpreter not initialized");
			return;
		}
		if (interpreter->Invoke() != kTfLiteOk){
			RCLCPP_ERROR( this->get_logger(), "ERROR: Something went wrong while invoking the interpreter...");
			return;
		}
		float* boxes = interpreter->typed_output_tensor<float>(0);
		float* confidence = interpreter->typed_output_tensor<float>(1);

	
		std::vector<size_t> filtered_indices = nonMaxSuppression(boxes, confidence, 2535, 0.4, 0.65);
		//RCLCPP_INFO(this->get_logger(), "Found %ld people", filtered_indices.size());

		cv::Mat boxes_img = img.clone();

		float scaleX = (float) img.cols / (float) DETECTION_MODEL_WIDTH[detection_model_n];
		float scaleY = (float) img.rows / (float) DETECTION_MODEL_HEIGHT[detection_model_n];

		size_t clients_index = 0;

		std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture> current_futures = {};

		for(size_t i: filtered_indices){

			float x1 = boxes[4 * i]     - boxes[4 * i + 2]/2.0;
			float y1 = boxes[4 * i + 1] - boxes[4 * i + 3]/2.0;
			float x2 = boxes[4 * i]     + boxes[4 * i + 2]/2.0;
			float y2 = boxes[4 * i + 1] + boxes[4 * i + 3]/2.0;

			cv::rectangle(boxes_img, cv::Point(static_cast<int>(x1 * scaleX), static_cast<int>(y1 * scaleY)), 
				cv::Point(static_cast<int>(x2 * scaleX), static_cast<int>(y2 * scaleY)),
				cv::Scalar(0, 255, 0), 2);

			cv::Rect box(x1*scaleX, y1*scaleY, (int)(boxes[4 * i + 2] * scaleX), (int)(boxes[4 * i + 3] * scaleY));

			box.x = std::max(0, box.x);
			box.y = std::max(0, box.y);
			box.width = std::min(box.width, img.cols - box.x);
			box.height = std::min(box.height, img.rows - box.y);

			cv::Mat crop = img(box);
			cv::resize(crop, small, cv::Size(MODEL_WIDTH[hpe_model_n], MODEL_HEIGHT[hpe_model_n]), cv::INTER_LINEAR);
			cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, small);
			cv_image_msg_bridge.toImageMsg(small_msg);

			auto det_msg = hpe_msgs::msg::Detection();
			auto box_msg = hpe_msgs::msg::Box();
			auto request = std::make_shared<hpe_msgs::srv::Estimate::Request>();


			box_msg.x 			= box.x;
			box_msg.y 			= box.y;
			box_msg.width 		= box.width;
			box_msg.height 		= box.height;
			box_msg.img_width 	= img.cols;
			box_msg.img_height 	= img.rows;

			det_msg.image 		= small_msg;
			det_msg.box 		= box_msg;

			request->detection 	= det_msg;


			if (clients_index >= clients_.size()) {
				RCLCPP_WARN(this->get_logger(), "worker_%s_%ld was not created yet, i will create a new one...", node_name.c_str(), clients_index);
    			(void) std::async(std::launch::async, &SlaveNode::create_new_worker, this);
			}else{
				if(clients_[clients_index]->wait_for_service(std::chrono::seconds(0))){
					current_futures.push_back(clients_[clients_index]->async_send_request(request).share());
				}else{
					RCLCPP_WARN(this->get_logger(), "worker_%s_%ld, is not ready yet...", node_name.c_str(), clients_index);
				}
			}
			clients_index++;
		}

		if(current_futures.size() > 0)
		{
			std::lock_guard<std::mutex> lock(queue_mutex_);
			futures_vector_queue_.push(current_futures);
		}
		
		cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, boxes_img);
		cv_image_msg_bridge.toImageMsg(small_msg);
		publisher_boxes_->publish(small_msg);
		double delay = (this->get_clock()->now() - start).seconds();
        avg_delay = (avg_delay*delay_window + delay);
        delay_window++;
        avg_delay /= delay_window;
	}

	void SlaveNode::loop(){
		running_ = true;
        rclcpp::Rate loop_rate(30); 

		std::vector<rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture> current_futures;
        while (running_)
        {
			current_futures = {};
			auto start = this->get_clock()->now();
			
			{
				std::unique_lock<std::mutex> lock(queue_mutex_);

				if (futures_vector_queue_.empty()) {
					//RCLCPP_WARN(this->get_logger(), "Future queue is empty, skipping this loop iteration.");
					continue;
				}

				RCLCPP_WARN(this->get_logger(), "Future queue size: %ld", futures_vector_queue_.size());
				current_futures = futures_vector_queue_.front();
				futures_vector_queue_.pop();
			}

	
			std_msgs::msg::Header header = std_msgs::msg::Header();
            std::vector<hpe_msgs::msg::Joints2d> all_joints = {};

			RCLCPP_WARN(this->get_logger(), "E, total: %ld", current_futures.size());

			for (size_t i = 0; i < current_futures.size(); ++i){
				RCLCPP_WARN(this->get_logger(), "E %ld", i);
				auto result = current_futures[i].wait_for(std::chrono::milliseconds(1500));
				if(result == std::future_status::ready){	
					try{
						auto response = current_futures[i].get();
						all_joints.push_back(response->hpe2d.joints);
						header = response->hpe2d.header;
					}
					catch(const std::exception& e){
						RCLCPP_ERROR(this->get_logger(), "Failed to get response from worker_%s_%zu", node_name.c_str(), i);
					}
				}else {
				    RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for response from worker_%s_%zu", node_name.c_str(), i);
				}
			}

			RCLCPP_WARN(this->get_logger(), "F");

			hpe_msgs::msg::Slave slave_msg = hpe_msgs::msg::Slave();
			slave_msg.header = header;
			slave_msg.all_joints = all_joints;
			slave_msg.skeletons_n = all_joints.size();

			RCLCPP_WARN(this->get_logger(), "G");

			publisher_slave_->publish(slave_msg);
			double delay = (this->get_clock()->now() - start).seconds();
			avg_delay_loop = (avg_delay_loop*delay_window_loop + delay);
			delay_window_loop++;
			avg_delay_loop /= delay_window_loop;
			loop_rate.sleep();
        }
	}

	void SlaveNode::setupTensors() {
		RCLCPP_INFO(this->get_logger(), "Building FlatBufferModel...");
		model = tflite::FlatBufferModel::BuildFromFile(DETECTION_MODEL_FILES[detection_model_n].c_str());
		if (!model) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to load model from file: %s", DETECTION_MODEL_FILES[detection_model_n].c_str());
			return;
		}

		tflite::ops::builtin::BuiltinOpResolver resolver;
		tflite::InterpreterBuilder builder(*model, resolver);

		RCLCPP_INFO(this->get_logger(), "Building interpreter...");
		builder(&interpreter); 
		
		if (!interpreter) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: could not create interpreter...");
			return;
		}
	

		auto status = interpreter->AllocateTensors();
		if (status != kTfLiteOk) {
			RCLCPP_ERROR(this->get_logger(), "ERROR ALLOCATING TENSORS!");
		}

		input_size = 1;

		int num_inputs = interpreter->inputs().size();
    	for (int i = 0; i < num_inputs; ++i) {
        	const TfLiteTensor* input_tensor = interpreter->tensor(interpreter->inputs()[i]);
        	RCLCPP_INFO(this->get_logger(), "Input Tensor %d: Type=%d", i, input_tensor->type);
        	RCLCPP_INFO(this->get_logger(), "Input Tensor %d Dimensions: ", i);
			for (int j = 0; j < input_tensor->dims->size; ++j) {
				input_size *= input_tensor->dims->data[j];
				RCLCPP_INFO(this->get_logger(), "Dim %d: %d", j, input_tensor->dims->data[j]);
			}
		}

		int num_outputs = interpreter->outputs().size();
		for (int i = 0; i < num_outputs; ++i) {
			const TfLiteTensor* output_tensor = interpreter->tensor(interpreter->outputs()[i]);
			RCLCPP_INFO(this->get_logger(), "Output Tensor %d: Type=%d", i, output_tensor->type);
			RCLCPP_INFO(this->get_logger(), "Output Tensor %d Dimensions: ", i);
			for (int j = 0; j < output_tensor->dims->size; ++j) {
				RCLCPP_INFO(this->get_logger(), "Dim %d: %d", j, output_tensor->dims->data[j]);
			}
		}

		input_tensor_idx = interpreter->inputs()[0];
		output_tensor_idx = interpreter->outputs()[0];

		input_dims = interpreter->tensor(input_tensor_idx)->dims;
		output_dims = interpreter->tensor(output_tensor_idx)->dims;


		input_data = interpreter->typed_input_tensor<float>(0);
	}

	void SlaveNode::callback(const sensor_msgs::msg::Image &msg) {
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception &e) {

			RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s", e.what());
		}

		findPpl(cv_ptr->image, msg.header);
	}

	

	SlaveNode::SlaveNode(std::string name, std::string raw_topic, int hpe_model_n_, int detection_model_n_, int starting_workers) : Node(name) {
		node_name = name;
		
		avg_delay = 0.0;
		avg_delay_loop = 0.0;
		delay_window = 0;
		delay_window_loop = 0;

		futures_vector_queue_={};

		hpe_model_n = hpe_model_n_;
		detection_model_n = detection_model_n_;

		if(hpe_model_n < 0 || (size_t) hpe_model_n > MODEL_FILES.size()){
			RCLCPP_ERROR(this->get_logger(), "NOT A GOOD HPE MODEL NUMBER. MUST BE BETWEEN 0 AND %ld", MODEL_FILES.size() - 1);
		}

		if(detection_model_n < 0 || (size_t) detection_model_n > DETECTION_MODEL_FILES.size()){
			RCLCPP_ERROR(this->get_logger(), "NOT A GOOD DETECTION MODEL NUMBER. MUST BE BETWEEN 0 AND %ld", DETECTION_MODEL_FILES.size() - 1);
		}

		clients_ = {};
		publisher_boxes_ = this->create_publisher<sensor_msgs::msg::Image>("/boxes_" + name, 10);
		publisher_slave_ = this->create_publisher<hpe_msgs::msg::Slave>("/slave_" + name, 10);

		for(int i = 0; i < starting_workers; i++){
			create_new_worker();
		}

		calibration_service_ = this->create_service<hpe_msgs::srv::Calibration>("/calibration_" + name, std::bind(&SlaveNode::calibrationService, this, _1, _2));
		setupTensors();

		if (raw_topic != "null") {
			subscription_ = this->create_subscription<sensor_msgs::msg::Image>(raw_topic, 10, std::bind(&SlaveNode::callback, this, _1));
			RCLCPP_INFO(this->get_logger(), "Ready");
		} else {
			RCLCPP_INFO(this->get_logger(), "Realsense camera not found, using webcam...");
			webcam_thread = std::thread([name, this]() {
				webcam_node = std::make_shared<hpe_test::WebcamNode>(name);
				rclcpp::spin(webcam_node);
			});
			subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/webcam_" + name, 10, std::bind(&SlaveNode::callback, this, _1));
		}

		loop_thread = std::thread(&SlaveNode::loop, this);
		loop_thread.detach();
  	}

	void SlaveNode::shutdown(){
		RCLCPP_INFO(this->get_logger(), "Shutting down...");
		running_ = false;
		if(webcam_node){
			webcam_node->stop();
		}

		if(webcam_thread.joinable()){
			webcam_thread.join();
			std::cout << "Webcam node joined..." << std::endl;
		}

		if(loop_thread.joinable()){
			loop_thread.join();
			std::cout << "Loop thread joined..." << std::endl;
		}

		for (auto& t : worker_threads) {
			if (t.joinable()) {
				t.join();
				std::cout << "Worker node joined..." << std::endl;
			}
		}
		RCLCPP_WARN(this->get_logger(), "Average FPS boxes: %f", 1 / avg_delay);
		RCLCPP_WARN(this->get_logger(), "Average FPS loop: %f", 1 / avg_delay_loop);
	}

	SlaveNode::~SlaveNode(){
		if(webcam_node){
			webcam_node->stop();
		}

		running_ = false;
		
		if(webcam_thread.joinable()){
			webcam_thread.join();
		}
	
		if(loop_thread.joinable()){
			loop_thread.join();
		}

		for (auto& t : worker_threads) {
			if (t.joinable()) {
				t.join();
			}
		}
	}

	void SlaveNode::calibrationService(const std::shared_ptr<hpe_msgs::srv::Calibration::Request> request [[maybe_unused]], std::shared_ptr<hpe_msgs::srv::Calibration::Response> response){

		std::ifstream file("camera_config.json");
		if (!file.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN CONFIGURATION FILE!");
			return;
		}

		nlohmann::json json_data;
		file >> json_data;

		auto intrinsic_json = json_data["camera"]["intrinsic"];

		std::vector<double> intrinsic(9);
		intrinsic[0] = intrinsic_json[0][0];  // fx
		intrinsic[1] = intrinsic_json[1][0];  // 0
		intrinsic[2] = intrinsic_json[2][0];  // 0

		intrinsic[3] = intrinsic_json[0][1];  // 0
		intrinsic[4] = intrinsic_json[1][1];  // fy
		intrinsic[5] = intrinsic_json[2][1];  // 0

		intrinsic[6] = intrinsic_json[0][2];  // cx
		intrinsic[7] = intrinsic_json[1][2];  // cy
		intrinsic[8] = intrinsic_json[2][2];  // 1

		hpe_msgs::msg::IntrinsicParams intr_prms = hpe_msgs::msg::IntrinsicParams();
		intr_prms.camera_matrix = intrinsic;

		auto distortion_json = json_data["camera"]["distortion"];
		std::vector<double> distortion(5);

		for(int i = 0; i < 5; i++)
			distortion[i] = distortion_json[i];

		intr_prms.distortion_coefficients = distortion;

		response->calibration.intrinsic_params = intr_prms;
		
		auto extrinsic_json = json_data["camera"]["extrinsic"];
		geometry_msgs::msg::TransformStamped transform_msg;
		
		transform_msg.transform.translation.x = extrinsic_json[0][3];  // t1
		transform_msg.transform.translation.y = extrinsic_json[1][3];  // t2
		transform_msg.transform.translation.z = extrinsic_json[2][3];  // t3

		// Convert the rotation matrix (top-left 3x3) to a quaternion
		tf2::Matrix3x3 rotation_matrix(
			extrinsic_json[0][0], extrinsic_json[0][1], extrinsic_json[0][2],
			extrinsic_json[1][0], extrinsic_json[1][1], extrinsic_json[1][2],
			extrinsic_json[2][0], extrinsic_json[2][1], extrinsic_json[2][2]
		);

		tf2::Quaternion quaternion;
    	rotation_matrix.getRotation(quaternion);

		transform_msg.transform.rotation.x = quaternion.x();
		transform_msg.transform.rotation.y = quaternion.y();
		transform_msg.transform.rotation.z = quaternion.z();
		transform_msg.transform.rotation.w = quaternion.w();

		transform_msg.header.frame_id = node_name;
		transform_msg.header.stamp = this->get_clock()->now();
		transform_msg.child_frame_id = "world";

		response->calibration.frame = transform_msg;
	}

};