#include <hpe_test/slave_node_single.hpp>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hpe_test {

void SlaveNodeSingle::setupTensors() {
  RCLCPP_INFO(this->get_logger(), "Building FlatBufferModel...");
  model =
      tflite::FlatBufferModel::BuildFromFile(MODEL_FILES[hpe_model_n].c_str());
  if (!model) {

    RCLCPP_ERROR(this->get_logger(),
                 "ERROR: Failed to load model from file: %s",
                 MODEL_FILES[hpe_model_n].c_str());
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
    const TfLiteTensor *input_tensor =
        interpreter->tensor(interpreter->inputs()[i]);
    RCLCPP_INFO(this->get_logger(), "Input Tensor %d: Type=%d", i,
                input_tensor->type);
    RCLCPP_INFO(this->get_logger(), "Input Tensor %d Dimensions: ", i);
    for (int j = 0; j < input_tensor->dims->size; ++j) {
      input_size *= input_tensor->dims->data[j];
      RCLCPP_INFO(this->get_logger(), "Dim %d: %d", j,
                  input_tensor->dims->data[j]);
    }
  }

  int num_outputs = interpreter->outputs().size();
  for (int i = 0; i < num_outputs; ++i) {
    const TfLiteTensor *output_tensor =
        interpreter->tensor(interpreter->outputs()[i]);
    RCLCPP_INFO(this->get_logger(), "Output Tensor %d: Type=%d", i,
                output_tensor->type);
    RCLCPP_INFO(this->get_logger(), "Output Tensor %d Dimensions: ", i);
    for (int j = 0; j < output_tensor->dims->size; ++j) {
      RCLCPP_INFO(this->get_logger(), "Dim %d: %d", j,
                  output_tensor->dims->data[j]);
    }
  }

  input_data = interpreter->typed_input_tensor<uint8_t>(0);
  output_data = interpreter->typed_output_tensor<float>(0);
}

void SlaveNodeSingle::callback(const sensor_msgs::msg::Image &msg) {
  auto start = this->get_clock()->now();

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {

    RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s", e.what());
  }

  cv::Mat small;
  cv::resize(cv_ptr->image, small,
             cv::Size(MODEL_WIDTH[hpe_model_n], MODEL_HEIGHT[hpe_model_n]),
             cv::INTER_LINEAR);

  memcpy(input_data, small.data, input_size * sizeof(uint8_t));

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

  for (size_t i = 0; i < out[0].size(); i++) {
    out[0][i] = (out[0][i] * (float)cv_ptr->image.rows);
    out[1][i] = (out[1][i] * (float)cv_ptr->image.cols);
  }

  auto joints = hpe_msgs::msg::Joints2d();

  joints.y = out[0];
  joints.x = out[1];
  joints.confidence = out[2];
  joints.dim = output_dims->data[2];

  std::vector<hpe_msgs::msg::Joints2d> all_joints = {};
  all_joints.push_back(joints);

  auto slave_msg = hpe_msgs::msg::Slave();

  slave_msg.header = msg.header;
  slave_msg.all_joints = all_joints;
  slave_msg.skeletons_n = all_joints.size();
  publisher_slave_->publish(slave_msg);

  cv_image_msg_bridge = cv_bridge::CvImage(
      msg.header, sensor_msgs::image_encodings::BGR8, cv_ptr->image);
  cv_image_msg_bridge.toImageMsg(small_msg);
  publisher_boxes_->publish(small_msg);

  double delay = (this->get_clock()->now() - start).seconds();
  avg_delay = (avg_delay * delay_window + delay);
  delay_window++;
  avg_delay /= delay_window;

  // RCLCPP_INFO(this->get_logger(), "Done 1 image.");
}

void SlaveNodeSingle::saveCameraInfo(const sensor_msgs::msg::CameraInfo &msg) {
  camera_info = msg;
}

void SlaveNodeSingle::compressedCallback(
    const sensor_msgs::msg::CompressedImage &msg) {
  cv::Mat image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
  sensor_msgs::msg::Image img_msg;
  img_msg.header = msg.header;

  // piccolo trick per leggere le bags, TODO fixare
  img_msg.header.stamp = this->get_clock()->now();

  img_msg.height = image.rows;
  img_msg.width = image.cols;
  img_msg.encoding = "bgr8";
  img_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  img_msg.data.assign(image.data,
                      image.data + image.total() * image.elemSize());

  callback(img_msg);
}

SlaveNodeSingle::SlaveNodeSingle(std::string name, std::string raw_topic,
                                 int hpe_model_n_,
                                 std::string calibration_topic)
    : Node(name), tf_buffer(this->get_clock()), tf_listener(tf_buffer) {
  node_name = name;
  avg_delay = 0.0;
  delay_window = 0;
  hpe_model_n = hpe_model_n_;

  if (hpe_model_n < 0 || (size_t)hpe_model_n > MODEL_FILES.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "NOT A GOOD HPE MODEL NUMBER. MUST BE BETWEEN 0 AND %ld",
                 MODEL_FILES.size() - 1);
  }

  calibration_from_json = calibration_topic.size() <= 0;

  if (!calibration_from_json)
    subscription_info_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            calibration_topic, 10,
            std::bind(&SlaveNodeSingle::saveCameraInfo, this, _1));

  publisher_boxes_ =
      this->create_publisher<sensor_msgs::msg::Image>("/boxes_" + name, 10);
  publisher_slave_ =
      this->create_publisher<hpe_msgs::msg::Slave>("/slave_" + name, 10);

  calibration_service_ = this->create_service<hpe_msgs::srv::Calibration>(
      "/calibration_" + name,
      std::bind(&SlaveNodeSingle::calibrationService, this, _1, _2));

  setupTensors();

  if (raw_topic != "null") {

    // maledette bags...
    auto topics = this->get_topic_names_and_types();
    if (topics.find(raw_topic) != topics.end()) {
      for (const auto &topic_type : topics[raw_topic])
        if (topic_type == "sensor_msgs/msg/Image") {
          subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
              raw_topic, 10, std::bind(&SlaveNodeSingle::callback, this, _1));
          RCLCPP_INFO(this->get_logger(),
                      "Ready, raw topic is sensor_msgs/msg/Imag");
          break;
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          compressed_image_sub_ =
              this->create_subscription<sensor_msgs::msg::CompressedImage>(
                  raw_topic, 10,
                  std::bind(&SlaveNodeSingle::compressedCallback, this, _1));
          RCLCPP_INFO(this->get_logger(),
                      "Ready, raw topic is sensor_msgs/msg/CompressedImage");
          break;
        } else
          RCLCPP_ERROR(this->get_logger(),
                       "%s is not a supported topic type for images",
                       topic_type.c_str());

    } else
      RCLCPP_ERROR(this->get_logger(), "%s not found", raw_topic.c_str());

  } else {
    RCLCPP_INFO(this->get_logger(), "Using webcam...");
    webcam_thread = std::thread([name, this]() {
      webcam_node = std::make_shared<hpe_test::WebcamNode>(name);
      rclcpp::spin(webcam_node);
    });
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/webcam_" + name, 10, std::bind(&SlaveNodeSingle::callback, this, _1));
  }
}

void SlaveNodeSingle::shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  if (webcam_node) {
    webcam_node->stop();
  }

  if (webcam_thread.joinable()) {
    webcam_thread.join();
    std::cout << "Webcam node joined..." << std::endl;
  }

  RCLCPP_WARN(this->get_logger(), "Average FPS: %f", 1 / avg_delay);
}

SlaveNodeSingle::~SlaveNodeSingle() {
  RCLCPP_WARN(this->get_logger(), "Average FPS: %f", 1 / avg_delay);
  if (webcam_node) {
    webcam_node->stop();
  }

  if (webcam_thread.joinable()) {
    webcam_thread.join();
  }
}

void SlaveNodeSingle::calibrationService(
    const std::shared_ptr<hpe_msgs::srv::Calibration::Request> request
    [[maybe_unused]],
    std::shared_ptr<hpe_msgs::srv::Calibration::Response> response) {

  RCLCPP_WARN(this->get_logger(), "Starting calibration service");
  if (calibration_from_json) {
    RCLCPP_WARN(this->get_logger(), "Using json");
    std::ifstream file("./calibration/calibration_" + node_name + ".json");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "FAILED TO OPEN CONFIGURATION FILE!");
      return;
    }

    nlohmann::json json_data;
    file >> json_data;

    auto intrinsic_json = json_data["camera"]["intrinsic"];

    std::vector<double> intrinsic(9);
    intrinsic[0] = intrinsic_json[0][0]; // fx
    intrinsic[1] = intrinsic_json[1][0]; // 0
    intrinsic[2] = intrinsic_json[2][0]; // 0

    intrinsic[3] = intrinsic_json[0][1]; // 0
    intrinsic[4] = intrinsic_json[1][1]; // fy
    intrinsic[5] = intrinsic_json[2][1]; // 0

    intrinsic[6] = intrinsic_json[0][2]; // cx
    intrinsic[7] = intrinsic_json[1][2]; // cy
    intrinsic[8] = intrinsic_json[2][2]; // 1

    hpe_msgs::msg::IntrinsicParams intr_prms = hpe_msgs::msg::IntrinsicParams();
    intr_prms.camera_matrix = intrinsic;

    auto distortion_json = json_data["camera"]["distortion"];
    std::vector<double> distortion(5);

    for (int i = 0; i < 5; i++)
      distortion[i] = distortion_json[i];

    intr_prms.distortion_coefficients = distortion;

    response->calibration.intrinsic_params = intr_prms;

    auto extrinsic_json = json_data["camera"]["extrinsic"];
    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.transform.translation.x = extrinsic_json[0][3]; // t1
    transform_msg.transform.translation.y = extrinsic_json[1][3]; // t2
    transform_msg.transform.translation.z = extrinsic_json[2][3]; // t3

    // Convert the rotation matrix (top-left 3x3) to a quaternion
    tf2::Matrix3x3 rotation_matrix(
        extrinsic_json[0][0], extrinsic_json[0][1], extrinsic_json[0][2],
        extrinsic_json[1][0], extrinsic_json[1][1], extrinsic_json[1][2],
        extrinsic_json[2][0], extrinsic_json[2][1], extrinsic_json[2][2]);

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
  } else {
    RCLCPP_WARN(this->get_logger(), "Using topic");
    hpe_msgs::msg::IntrinsicParams intr_prms = hpe_msgs::msg::IntrinsicParams();
    std::string frame_id = camera_info.header.frame_id;

    std::vector<double> distortion(5);
    std::vector<double> k(9);

    for (size_t i = 0; i < 5 && i < camera_info.d.size(); i++) {
      distortion[i] = camera_info.d[i];
    }

    for (size_t row = 0; row < 3; ++row) {
      for (size_t col = 0; col < 3; ++col) {
        k[col * 3 + row] = camera_info.k[row * 3 + col];
      }
    }

    intr_prms.distortion_coefficients = distortion;
    intr_prms.camera_matrix = k;

    response->calibration.frame =
        tf_buffer.lookupTransform("world", frame_id, tf2::TimePointZero);
    response->calibration.intrinsic_params = intr_prms;

    RCLCPP_WARN(this->get_logger(),
                "Calibratin frame id: %s \nMatrix k:\n[%f, %f, %f]\n[%f, %f, "
                "%f]\n[%f, %f, %f]\nMatrix k (column major):\n[%f, %f, %f, %f, "
                "%f, %f, %f, %f, %f]\n",
                frame_id.c_str(), camera_info.k[0], camera_info.k[1],
                camera_info.k[2], camera_info.k[3], camera_info.k[4],
                camera_info.k[5], camera_info.k[6], camera_info.k[7],
                camera_info.k[8], k[0], k[1], k[2], k[3], k[4], k[5], k[6],
                k[7], k[8]);

    RCLCPP_WARN(this->get_logger(), "Calibration finished");
  }
}

}; // namespace hpe_test
