#include <hpe_test/resizer_node.hpp>

using std::placeholders::_1;

namespace hpe_test {

	float ResizerNode::computeIoU(const float* box1, const float* box2) {
		float x1 = std::max(box1[0] - box1[2]/2.0, box2[0] - box2[2]/2.0);
		float y1 = std::max(box1[1] - box1[3]/2.0, box2[1] - box2[3]/2.0);
		float x2 = std::min(box1[0] + box1[2]/2.0, box2[0] + box2[2]/2.0);
		float y2 = std::min(box1[1] + box1[3]/2.0, box2[1] + box2[3]/2.0);

		float intersectionArea = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
		float box1Area = box1[2] * box1[3];
		float box2Area = box2[2] * box2[3];

		return intersectionArea / (box1Area + box2Area - intersectionArea);
	}

	std::vector<size_t> ResizerNode::nonMaxSuppression(float* boxes, float* scores, size_t numBoxes, float iouThreshold, float minConfidence) {
		std::vector<size_t> indices(numBoxes);
		std::vector<size_t> final_indices;

		std::iota(indices.begin(), indices.end(), 0);

		// Sort indices by score in descending order
		std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
			return scores[i1] > scores[i2];
		});

		std::vector<bool> suppressed(numBoxes, false);

		for (size_t i = 0; i < numBoxes; ++i) {
			size_t idx = indices[i];
			if (scores[idx] < minConfidence || suppressed[idx]) continue;
			final_indices.push_back(idx);

			//std::string out ="Box: (" + std::to_string(boxes[4 * idx]) + ", " + std::to_string(boxes[4 * idx + 1]) + ", " + std::to_string(boxes[4 * idx + 2]) + ", " +std::to_string(boxes[4 * idx + 3]) + ") with score " + std::to_string(scores[idx]);
			//RCLCPP_INFO(this->get_logger(), "%s", out.c_str());


			for (size_t j = i + 1; j < numBoxes; ++j) {
				size_t compareIdx = indices[j];
				if (computeIoU(boxes + 4 * idx, boxes + 4 * compareIdx) > iouThreshold) {
					suppressed[compareIdx] = true;
				}
			}
		}
		return final_indices;
	}

	void ResizerNode::create_new_worker(){

		std::string name = node_name + "_" + std::to_string(workers_n);
		publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>("/box/" + name, 10));
		
		if (publishers_[workers_n] == nullptr) {
			RCLCPP_ERROR(this->get_logger(), "Publisher is not initialized");
			return;
		}

		worker_threads.push_back(std::thread([name, this]() {
			auto worker_node = std::make_shared<hpe_test::WorkerNode>(name, model_file);
			rclcpp::spin(worker_node);
			rclcpp::shutdown();
		}));
		
		workers_n++;
	}

  	void ResizerNode::findPpl(cv::Mat &img, std_msgs::msg::Header header) {

		cv::resize(img, small, cv::Size(input_width, input_height), cv::INTER_LINEAR);

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

		float* boxes;
		float* confidence;


		for (size_t i = 0; i < interpreter->outputs().size(); i++) {
				
			float* output_data = interpreter->typed_output_tensor<float>(i);
			if(i == 0)
				boxes = output_data;
			else
				confidence = output_data;
		}
	
		std::vector<size_t> filtered_indices = nonMaxSuppression(boxes, confidence, 2535, 0.75, 0.75);

		cv::Mat boxes_img = img.clone();

		float scaleX = (float) img.cols / (float) input_width;
		float scaleY = (float) img.rows / (float) input_height;

		size_t publishers_index = 0;
		
		for(size_t i: filtered_indices){

			float x1 = boxes[4 * i]     - boxes[4 * i + 2]/2.0;
			float y1 = boxes[4 * i + 1] - boxes[4 * i + 3]/2.0;
			float x2 = boxes[4 * i]     + boxes[4 * i + 2]/2.0;
			float y2 = boxes[4 * i + 1] + boxes[4 * i + 3]/2.0;

			RCLCPP_INFO(this->get_logger(), "x1: %f, y1: %f, x2: %f, y2, %f, ScaleX: %f, scaleY: %f", x1, y1, x2, y2, scaleX, scaleY);

			cv::rectangle(boxes_img, cv::Point(static_cast<int>(x1 * scaleX), static_cast<int>(y1 * scaleY)), 
				cv::Point(static_cast<int>(x2 * scaleX), static_cast<int>(y2 * scaleY)),
				cv::Scalar(0, 255, 0), 2);

			cv::Rect box(x1*scaleX, y1*scaleY, (int)(boxes[4 * i + 2] * scaleX), (int)(boxes[4 * i + 3] * scaleY));

			box.x = std::max(0, box.x);
			box.y = std::max(0, box.y);
			box.width = std::min(box.width, img.cols - box.x);
			box.height = std::min(box.height, img.rows - box.y);

			cv::Mat crop = img(box);
			cv::resize(crop, small, cv::Size(width, height), cv::INTER_LINEAR);
			cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, small);
			cv_image_msg_bridge.toImageMsg(small_msg);

			if (publishers_[publishers_index] == nullptr) {
				RCLCPP_ERROR(this->get_logger(), "Publisher is not initialized for index %ld, i will allocate a new one...", i);
			}else{
				publishers_[publishers_index]->publish(small_msg);	
			}

			publishers_index++;

		}

		cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, boxes_img);
		cv_image_msg_bridge.toImageMsg(small_msg);		
		publisher_boxes_->publish(small_msg);
	}

	void ResizerNode::setupTensors() {
		RCLCPP_INFO(this->get_logger(), "Building FlatBufferModel...");
		model = tflite::FlatBufferModel::BuildFromFile(boxes_model_file.c_str());
		if (!model) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to load model from file: %s", boxes_model_file.c_str());
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

		RCLCPP_INFO(this->get_logger(), "Setting up dimensions...");



		input_tensor_idx = interpreter->inputs()[0];
		output_tensor_idx = interpreter->outputs()[0];

		input_dims = interpreter->tensor(input_tensor_idx)->dims;
		output_dims = interpreter->tensor(output_tensor_idx)->dims;
		const auto &input_tensor = interpreter->tensor(interpreter->inputs()[0]);
		const auto &output_tensor = interpreter->tensor(interpreter->outputs()[0]);

		inputType = input_tensor->type;

		switch (inputType) {
		case kTfLiteFloat32:
			RCLCPP_INFO(this->get_logger(), "Tensors input type is Float32");
		break;
		case kTfLiteFloat16:
			RCLCPP_INFO(this->get_logger(), "Tensors input type is Float32");
		break;
		case kTfLiteUInt8:
			RCLCPP_INFO(this->get_logger(), "Tensors input type is Uint8");
		break;
		default:
			RCLCPP_INFO(this->get_logger(), "Tensors input type is unknown...");
		}

		outputType = output_tensor->type;
		switch (outputType) {
		case kTfLiteFloat32:
			RCLCPP_INFO(this->get_logger(), "Tensors output type is Float32");
		break;
		case kTfLiteFloat16:
			RCLCPP_INFO(this->get_logger(), "Tensors output type is Float32");
		break;
		case kTfLiteUInt8:
			RCLCPP_INFO(this->get_logger(), "Tensors output type is Uint8");
		break;
		default:
			RCLCPP_INFO(this->get_logger(), "Tensors output type is unknown...");
		}

		input_size = 1;

		std::string in_d_st = "";
		int num_dims = input_dims->size;
		for (int i = 0; i < num_dims; i++) {
			int dim_size = input_dims->data[i];
			in_d_st += std::to_string(dim_size);
			input_size *= dim_size;
			if ((i + 1) < num_dims)
				in_d_st += "x";

			if (i == 1)
				input_height = dim_size;
			else if (i == 2)
				input_width = dim_size;
		}

		output_size = 1;
		std::string out_d_st = "";
		num_dims = output_dims->size;
		for (int i = 0; i < num_dims; i++) {
			int dim_size = output_dims->data[i];
			output_size *= dim_size;
			out_d_st += std::to_string(dim_size);
			if ((i + 1) < num_dims)
				out_d_st += "x";
		}

		input_data = interpreter->typed_input_tensor<float>(0);
		output_data = interpreter->typed_output_tensor<float>(0);

		RCLCPP_INFO(this->get_logger(), "Input dimensions are: %s, and the size is: %d", in_d_st.c_str(), input_size);
		RCLCPP_INFO(this->get_logger(), "Output dimensions are: %s, and the size is %d", out_d_st.c_str(), output_size);
	}

	void ResizerNode::callback(const sensor_msgs::msg::Image &msg) {
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception &e) {

			RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s", e.what());
		}

		findPpl(cv_ptr->image, msg.header);
	}

  	void ResizerNode::open_camera() {
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
				RCLCPP_ERROR(this->get_logger(), "ERROR: missin a frame...");
			}

			std_msgs::msg::Header header;
			header.set__stamp(this->get_clock()->now());
			header.set__frame_id(camera_frame);

			findPpl(frame, header);
		}

		cap.release();
	}

	ResizerNode::ResizerNode(std::string name, std::string raw_topic, int h, int w, int starting_workers) : Node("resizer_" + name) {
		node_name = name;
		width = w;
		height = h;
		publishers_ = {};
		publisher_boxes_ = this->create_publisher<sensor_msgs::msg::Image>("/boxes/" + name, 10);
		for(int i = 0; i < starting_workers; i++){
			create_new_worker();
		}
		camera_frame = name;
		setupTensors();
		if (raw_topic != "null") {
			subscription_ = this->create_subscription<sensor_msgs::msg::Image>(raw_topic, 10, std::bind(&ResizerNode::callback, this, _1));
			RCLCPP_INFO(this->get_logger(), "Ready");
		} else {
			RCLCPP_INFO(this->get_logger(), "Realsense camera not found, using webcam...");
		open_camera();
		}
  	}

	ResizerNode::~ResizerNode(){
		for (auto& t : worker_threads) {
			if (t.joinable()) {
				t.join();
			}
		}
	}
};