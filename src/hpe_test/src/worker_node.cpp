#include "hpe_test/worker_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


namespace hpe_test{
		
	void WorkerNode::service(const std::shared_ptr<hpe_msgs::srv::Estimate::Request> request, std::shared_ptr<hpe_msgs::srv::Estimate::Response> response) {
		auto start = this->get_clock()->now();
		try {
			cv_ptr = cv_bridge::toCvCopy(request->detection.image, sensor_msgs::image_encodings::BGR8);
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
		
		//Deal with warping...
		for(size_t i = 0; i < out[0].size(); i++){
			out[0][i] = (out[0][i]*(float)request->detection.box.height + (float)request->detection.box.y)/(float)request->detection.box.img_height;
			out[1][i] = (out[1][i]*(float)request->detection.box.width  + (float)request->detection.box.x)/(float)request->detection.box.img_width;
		}

		response->hpe2d.header = request->detection.image.header;
		response->hpe2d.joints.y = out[0];
		response->hpe2d.joints.x = out[1];
		response->hpe2d.joints.confidence = out[2];
		response->hpe2d.joints.dim = output_dims->data[2];
		
		double delay = (this->get_clock()->now() - start).seconds();
        avg_delay = (avg_delay*delay_window + delay);
        delay_window++;
        avg_delay /= delay_window;
		RCLCPP_INFO(this->get_logger(), "Service %d processing completed", delay_window);

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

		input_data = interpreter->typed_input_tensor<uint8_t>(0);
		output_data = interpreter->typed_output_tensor<float>(0);
	}

	WorkerNode::WorkerNode(std::string name, int model_): Node("worker_" + name) {
		hpe_model_n = model_;
		setupTensors();
		delay_window = 0;
		avg_delay = 0.0;
		service_ = this->create_service<hpe_msgs::srv::Estimate>("estimate" + name, std::bind(&WorkerNode::service, this, _1, _2));
		RCLCPP_INFO(this->get_logger(), "worker_%s is ready", name.c_str());
	}

	WorkerNode::~WorkerNode(){
		RCLCPP_WARN(this->get_logger(), "Average FPS: %f, total frames: %d", 1/avg_delay, delay_window);
	}
};

