#include "hpe_test/worker_node.hpp"

using std::placeholders::_1;

namespace hpe_test{
		
	void WorkerNode::callback(const sensor_msgs::msg::Image &msg) {
		RCLCPP_INFO(this->get_logger(), "Working on a new image");

		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception &e) {

			RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s",
									e.what());
		}

		// FILL input_data with image

		// magia del c++, se fa errori guardare qui
		// memcpy(input_data, &msg.data, input_size * sizeof(uint8_t));
		memcpy(input_data, cv_ptr->image.data, input_size * sizeof(uint8_t));

		if (interpreter->Invoke() != kTfLiteOk)
			RCLCPP_ERROR(
					this->get_logger(),
					"ERROR: Something went wrong while invoking the interpreter...");

		std::vector<std::vector<float>> out(output_dims->data[3]);

		for (int i = 0; i < output_dims->data[2]; i++) {
			for (int j = 0; j < output_dims->data[3]; j++) {
				out[j].push_back(output_data[i * output_dims->data[3] + j]);
			}
		}

		hpe_msg.joints_x = out[1];
		hpe_msg.joints_y = out[0];
		hpe_msg.confidence = out[2];
		hpe_msg.dim = output_dims->data[2];
		publisher_->publish(hpe_msg);
	}

	void WorkerNode::setupTensors() {
		RCLCPP_INFO(this->get_logger(), "Building FlatBufferModel...");
		model = tflite::FlatBufferModel::BuildFromFile(model_file.c_str());
		if (!model) {
			RCLCPP_ERROR(this->get_logger(),
										"ERROR: Failed to load model from file: %s",
										model_file.c_str());
			return;
		}

		tflite::ops::builtin::BuiltinOpResolver resolver;
		tflite::InterpreterBuilder builder(*model, resolver);
		RCLCPP_INFO(this->get_logger(), "Building interpreter...");
		builder(&interpreter);
		if (!interpreter) {
			RCLCPP_ERROR(this->get_logger(),
										"ERROR: could not create interpreter...");

			return;
		}

		TfLiteStatus status = interpreter->AllocateTensors();
		if (status != kTfLiteOk) {
			RCLCPP_ERROR(this->get_logger(), "ERROR ALLOCATING TENSORS!");
		}

		RCLCPP_INFO(this->get_logger(), "Setting up dimensions...");
		// interpreter->SetNumThreads(4);
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
		for (int i = 0; i < num_dims; ++i) {
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
		for (int i = 0; i < num_dims; ++i) {
			int dim_size = output_dims->data[i];
			output_size *= dim_size;
			out_d_st += std::to_string(dim_size);
			if ((i + 1) < num_dims)
				out_d_st += "x";
		}

		input_data = interpreter->typed_input_tensor<uint8_t>(0);
		output_data = interpreter->typed_output_tensor<float>(0);

		RCLCPP_INFO(this->get_logger(), "Input dimensions are: %s", in_d_st.c_str());
		RCLCPP_INFO(this->get_logger(), "Output dimensions are: %s", out_d_st.c_str());
	}

	WorkerNode::WorkerNode(std::string name, std::string model): Node("worker_" + name) {
		model_file = model;
		publisher_ = this->create_publisher<hpe_msgs::msg::Hpe2d>("/hpe_result/" + name, 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/box/" + name, 10, std::bind(&WorkerNode::callback, this, _1));
		setupTensors();
	}

	WorkerNode::~WorkerNode(){
		//TODO
	}
};

