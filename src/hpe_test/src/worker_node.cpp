#include "hpe_test/worker_node.hpp"

using std::placeholders::_1;

namespace hpe_test{
		
	void WorkerNode::callback(const hpe_msgs::msg::Detection &msg) {
		//RCLCPP_INFO(this->get_logger(), "Working on a new image");

		try {
			cv_ptr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception &e) {

			RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s",
									e.what());
		}

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

		hpe_msg.header = msg.image.header;
		hpe_msg.joints_x = out[1];
		hpe_msg.joints_y = out[0];
		hpe_msg.confidence = out[2];
		hpe_msg.dim = output_dims->data[2];
		hpe_msg.box = msg.box;
		publisher_->publish(hpe_msg);
	}

	void WorkerNode::setupTensors() {
		RCLCPP_INFO(this->get_logger(), "Building FlatBufferModel...");
		model = tflite::FlatBufferModel::BuildFromFile(MODEL_FILES[hpe_model_n].c_str());
		if (!model) {
			RCLCPP_ERROR(this->get_logger(),"ERROR: Failed to load model from file: %s", MODEL_FILES[hpe_model_n].c_str());
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
		
		int num_inputs = interpreter->inputs().size();
    	for (int i = 0; i < num_inputs; ++i) {
        	const TfLiteTensor* input_tensor = interpreter->tensor(interpreter->inputs()[i]);
        	RCLCPP_INFO(this->get_logger(), "Input Tensor %d: Type=%d", i, input_tensor->type);
        	RCLCPP_INFO(this->get_logger(), "Input Tensor %d Dimensions: ", i);
			for (int j = 0; j < input_tensor->dims->size; ++j) {
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

		input_data = interpreter->typed_input_tensor<uint8_t>(0);
		output_data = interpreter->typed_output_tensor<float>(0);
	}

	WorkerNode::WorkerNode(std::string name, int model_): Node("worker_" + name) {
		hpe_model_n = model_;
		setupTensors();
		publisher_ = this->create_publisher<hpe_msgs::msg::Hpe2d>("/hpe_result/" + name, 10);
		subscription_ = this->create_subscription<hpe_msgs::msg::Detection>("/box/" + name, 10, std::bind(&WorkerNode::callback, this, _1));
	}

	WorkerNode::~WorkerNode(){
		//TODO
	}
};

