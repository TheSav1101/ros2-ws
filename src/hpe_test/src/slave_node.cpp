#include "sensor_msgs/msg/compressed_image.hpp"
#include <hpe_test/slave_node.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace hpe_test {

sensor_msgs::msg::CompressedImage compress_image(const cv::Mat &img) {
  std::vector<uchar> buffer;
  std::vector<int> compression_params;

  compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};

  if (!cv::imencode(".jpeg", img, buffer, compression_params)) {
    throw std::runtime_error("Failed to encode image to .jpeg");
  }
  sensor_msgs::msg::CompressedImage compressed_image_msg;
  compressed_image_msg.format = "jpeg";
  compressed_image_msg.data = buffer;

  return compressed_image_msg;
}

float SlaveNode::computeIoU(const float *box1, const float *box2) {
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

std::vector<size_t> SlaveNode::nonMaxSuppression(const float *boxes,
                                                 const float *scores,
                                                 const size_t numBoxes,
                                                 const float iouThreshold,
                                                 const float minConfidence) {
  std::vector<size_t> indices(numBoxes);
  std::vector<size_t> final_indices;

  std::iota(indices.begin(), indices.end(), 0);

  std::sort(indices.begin(), indices.end(),
            [&](size_t i1, size_t i2) { return scores[i1] > scores[i2]; });

  std::vector<bool> suppressed(numBoxes, false);

  for (size_t i = 0; i < numBoxes; ++i) {
    size_t idx = indices[i];
    if (scores[idx] < minConfidence || suppressed[idx])
      continue;
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

void SlaveNode::create_new_worker() {

  std::string name = node_name + "_" + std::to_string(workers_n);
  workers_n++;

  clients_.push_back(
      this->create_client<hpe_msgs::srv::Estimate>("estimate" + name));

  auto new_w = std::make_shared<hpe_test::WorkerNode>(name, hpe_model_n);

  executor_->add_node(new_w);

  workers_.push_back(new_w);
}

void SlaveNode::callback(const sensor_msgs::msg::Image &msg) {
  cv::Mat img;
  std_msgs::msg::Header header;

  auto start = this->get_clock()->now();

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s", e.what());
  }

  img = cv_ptr->image;
  header = msg.header;

  cv::resize(img, small,
             cv::Size(DETECTION_MODEL_WIDTH[detection_model_n],
                      DETECTION_MODEL_HEIGHT[detection_model_n]),
             cv::INTER_LINEAR);

  // TODO togliere sta roba
  cv::Mat floatImg;
  small.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);

  memcpy(input_data, floatImg.data, input_size * sizeof(float));

  if (!interpreter) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: Interpreter not initialized");
    return;
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    RCLCPP_ERROR(
        this->get_logger(),
        "ERROR: Something went wrong while invoking the interpreter...");
    return;
  }
  const float *boxes = interpreter->typed_output_tensor<float>(0);
  const float *confidence = interpreter->typed_output_tensor<float>(1);

  std::vector<size_t> filtered_indices =
      nonMaxSuppression(boxes, confidence, 2535, 0.25, 0.35);

  cv::Mat boxes_img = img.clone();

  float scaleX =
      (float)img.cols / (float)DETECTION_MODEL_WIDTH[detection_model_n];
  float scaleY =
      (float)img.rows / (float)DETECTION_MODEL_HEIGHT[detection_model_n];

  size_t clients_index = 0;

  Responses *curr_responses = new Responses(filtered_indices.size());

  responses_ptrs_.push_back(curr_responses);

  auto response_received_callback =
      [this, curr_responses](
          rclcpp::Client<hpe_msgs::srv::Estimate>::SharedFuture future) {
        // RCLCPP_INFO(this->get_logger(), "Executing callback");
        auto result = future.get();
        if (curr_responses->add(result->hpe2d)) {
          all_response_received_callback(curr_responses);
        }
      };

  for (size_t i : filtered_indices) {

    float x1 = boxes[4 * i] - boxes[4 * i + 2] / 2.0;
    float y1 = boxes[4 * i + 1] - boxes[4 * i + 3] / 2.0;
    float x2 = boxes[4 * i] + boxes[4 * i + 2] / 2.0;
    float y2 = boxes[4 * i + 1] + boxes[4 * i + 3] / 2.0;

    cv::rectangle(
        boxes_img,
        cv::Point(static_cast<int>(x1 * scaleX), static_cast<int>(y1 * scaleY)),
        cv::Point(static_cast<int>(x2 * scaleX), static_cast<int>(y2 * scaleY)),
        cv::Scalar(0, 255, 0), 2);

    cv::Rect box(x1 * scaleX, y1 * scaleY, (int)(boxes[4 * i + 2] * scaleX),
                 (int)(boxes[4 * i + 3] * scaleY));

    box.x = std::max(0, box.x);
    box.y = std::max(0, box.y);
    box.width = std::min(box.width, img.cols - box.x);
    box.height = std::min(box.height, img.rows - box.y);

    cv::Mat crop = img(box);
    cv::resize(crop, small,
               cv::Size(MODEL_WIDTH[hpe_model_n], MODEL_HEIGHT[hpe_model_n]),
               cv::INTER_LINEAR);

    cv_image_msg_bridge =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, small);
    cv_image_msg_bridge.toImageMsg(small_msg);

    auto det_msg = hpe_msgs::msg::Detection();
    auto box_msg = hpe_msgs::msg::Box();
    auto request = std::make_shared<hpe_msgs::srv::Estimate::Request>();

    box_msg.x = box.x;
    box_msg.y = box.y;
    box_msg.width = box.width;
    box_msg.height = box.height;
    box_msg.img_width = img.cols;
    box_msg.img_height = img.rows;

    det_msg.image = small_msg;
    det_msg.box = box_msg;

    request->detection = det_msg;
    request->request_number = delay_window;

    if (clients_index >= clients_.size()) {
      RCLCPP_WARN(
          this->get_logger(),
          "worker_%s_%ld was not created yet, i will create a new one...",
          node_name.c_str(), clients_index);
      auto future =
          std::async(std::launch::async, &SlaveNode::create_new_worker, this);
    } else {
      if (clients_[clients_index]->wait_for_service(std::chrono::seconds(0)) &&
          workers_[clients_index]->isReady()) {
        clients_[clients_index]->async_send_request(request,
                                                    response_received_callback);
      } else {
        RCLCPP_WARN(this->get_logger(), "worker_%s_%ld, is not ready yet...",
                    node_name.c_str(), clients_index);
      }
    }
    clients_index++;
  }

  publisher_boxes_->publish(compress_image(boxes_img));
  double delay = (this->get_clock()->now() - start).seconds();
  avg_delay = (avg_delay * delay_window + delay);
  delay_window++;
  avg_delay /= delay_window;
  RCLCPP_INFO(this->get_logger(), "%d: ppl found=%ld, FPS=%f", delay_window,
              filtered_indices.size(), 1 / delay);
}

void SlaveNode::all_response_received_callback(Responses *response) {

  if (response->isReady()) {
    std_msgs::msg::Header header = std_msgs::msg::Header();
    std::vector<hpe_msgs::msg::Joints2d> all_joints = {};

    for (hpe_msgs::msg::Hpe2d hpe : response->get()) {
      all_joints.push_back(hpe.joints);
      header = hpe.header;
    }

    hpe_msgs::msg::Slave slave_msg = hpe_msgs::msg::Slave();
    slave_msg.header = header;
    slave_msg.all_joints = all_joints;
    slave_msg.skeletons_n = all_joints.size();

    publisher_slave_->publish(slave_msg);

    // delete pointer
    responses_ptrs_.erase(
        find(responses_ptrs_.begin(), responses_ptrs_.end(), response));
    delete response;

    double delay = (this->get_clock()->now() - header.stamp).seconds();
    avg_delay_total = (avg_delay_total * delay_window_total + delay);
    delay_window_total++;
    avg_delay_total /= delay_window_total;
  }
}

void SlaveNode::setupTensors() {
  model = tflite::FlatBufferModel::BuildFromFile(
      DETECTION_MODEL_FILES[detection_model_n].c_str());
  if (!model) {
    RCLCPP_ERROR(this->get_logger(),
                 "ERROR: Failed to load model from file: %s",
                 DETECTION_MODEL_FILES[detection_model_n].c_str());
    return;
  }

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model, resolver);

  if (gpu_acceleration) {
    RCLCPP_WARN(this->get_logger(), "Using gpu delegate.");
    auto *delegate = TfLiteGpuDelegateV2Create(/*default options=*/nullptr);
    builder.AddDelegate(delegate);
  } else {
    builder.AddDelegate((TfLiteDelegate *)nullptr);
  }

  builder(&interpreter, 4);

  if (!interpreter) {
    RCLCPP_ERROR(this->get_logger(), "ERROR: could not create interpreter...");
    return;
  }

  if (!gpu_acceleration) {
    RCLCPP_WARN(this->get_logger(), "Using default delegate.");
    interpreter->SetNumThreads(4);
    RCLCPP_WARN(this->get_logger(), "Done");
  }

  auto status = interpreter->AllocateTensors();
  if (status != kTfLiteOk) {
    RCLCPP_ERROR(this->get_logger(), "ERROR ALLOCATING TENSORS!");
  }

  input_size = 1;

  int num_inputs = interpreter->inputs().size();
  for (int i = 0; i < num_inputs; ++i) {
    const TfLiteTensor *input_tensor =
        interpreter->tensor(interpreter->inputs()[i]);
    for (int j = 0; j < input_tensor->dims->size; ++j) {
      input_size *= input_tensor->dims->data[j];
    }
  }

  input_tensor_idx = interpreter->inputs()[0];
  output_tensor_idx = interpreter->outputs()[0];

  input_dims = interpreter->tensor(input_tensor_idx)->dims;
  output_dims = interpreter->tensor(output_tensor_idx)->dims;

  input_data = interpreter->typed_input_tensor<float>(0);
}

void SlaveNode::compressedCallback(
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

void SlaveNode::saveCameraInfo(const sensor_msgs::msg::CameraInfo &msg) {
  camera_info = msg;
}

SlaveNode::SlaveNode(rclcpp::executors::MultiThreadedExecutor *executor,
                     std::string name, std::string raw_topic, int hpe_model_n_,
                     int detection_model_n_, int starting_workers,
                     std::string calibration_topic, int gpu_support)
    : Node(name), tf_buffer(this->get_clock()), tf_listener(tf_buffer) {

  node_name = name;
  executor_ = executor;

  if (gpu_support) {
    gpu_acceleration = true;
  } else {
    gpu_acceleration = false;
  }

  responses_ptrs_ = {};

  avg_delay = 0.0;
  delay_window = 0;
  avg_delay_total = 0.0;
  delay_window_total = 0;

  hpe_model_n = hpe_model_n_;
  detection_model_n = detection_model_n_;

  calibration_from_json = calibration_topic.size() <= 0;

  if (!calibration_from_json)
    subscription_info_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            calibration_topic, 10,
            std::bind(&SlaveNode::saveCameraInfo, this, _1));

  if (hpe_model_n < 0 || (size_t)hpe_model_n > MODEL_FILES.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "NOT A GOOD HPE MODEL NUMBER. MUST BE BETWEEN 0 AND %ld",
                 MODEL_FILES.size() - 1);
  }

  if (detection_model_n < 0 ||
      (size_t)detection_model_n > DETECTION_MODEL_FILES.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "NOT A GOOD DETECTION MODEL NUMBER. MUST BE BETWEEN 0 AND %ld",
                 DETECTION_MODEL_FILES.size() - 1);
  }

  clients_ = {};
  publisher_boxes_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/boxes_" + name, 10);
  publisher_slave_ =
      this->create_publisher<hpe_msgs::msg::Slave>("/slave_" + name, 10);

  for (int i = 0; i < starting_workers; i++) {
    create_new_worker();
  }

  calibration_service_ = this->create_service<hpe_msgs::srv::Calibration>(
      "/calibration_" + name,
      std::bind(&SlaveNode::calibrationService, this, _1, _2));

  setupTensors();

  if (raw_topic != "null") {
    // maledette bags...
    auto topics = this->get_topic_names_and_types();
    if (topics.find(raw_topic) != topics.end()) {
      for (const auto &topic_type : topics[raw_topic])
        if (topic_type == "sensor_msgs/msg/Image") {
          subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
              raw_topic, 10, std::bind(&SlaveNode::callback, this, _1));
          RCLCPP_INFO(this->get_logger(),
                      "Ready, raw topic is sensor_msgs/msg/Imag");
          break;
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          compressed_image_sub_ =
              this->create_subscription<sensor_msgs::msg::CompressedImage>(
                  raw_topic, 10,
                  std::bind(&SlaveNode::compressedCallback, this, _1));
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
    RCLCPP_INFO(this->get_logger(),
                "Realsense camera not found, using webcam...");

    webcam_node = std::make_shared<hpe_test::WebcamNode>(name);

    executor_->add_node(webcam_node);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/webcam_" + name, 10, std::bind(&SlaveNode::callback, this, _1));
  }
}

void SlaveNode::shutdown() {
  RCLCPP_WARN(this->get_logger(), "Average FPS boxes: %f, total frames: %d",
              1 / avg_delay, delay_window);
  RCLCPP_WARN(this->get_logger(), "Average FPS total: %f, total frames: %d",
              1 / avg_delay_total, delay_window_total);
  RCLCPP_INFO(this->get_logger(), "Shutting down...");
  running_ = false;

  if (webcam_node) {
    webcam_node->stop();
    executor_->remove_node(webcam_node);
  }

  for (auto &w : workers_) {
    w->shutdown();
    w.reset();
    executor_->remove_node(w);
  }

  RCLCPP_WARN(this->get_logger(), "Average FPS boxes: %f, total frames: %d",
              1 / avg_delay, delay_window);
  RCLCPP_WARN(this->get_logger(), "Average total delay: %f, total frames: %d",
              avg_delay_total, delay_window_total);
}

SlaveNode::~SlaveNode() {

  running_ = false;

  if (webcam_node) {
    webcam_node->stop();
    executor_->remove_node(webcam_node);
  }

  for (auto &w : workers_) {
    w->shutdown();
    executor_->remove_node(w);
  }

  for (auto p : responses_ptrs_) {
    delete p;
  }
}

void SlaveNode::calibrationService(
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
