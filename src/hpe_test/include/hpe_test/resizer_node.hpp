#ifndef HPE_TEST__RESIZER_NODE_HPP_
#define HPE_TEST__RESIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <vector>
#include "hpe_msgs/msg/hpe2d.hpp"
#include "hpe_test/worker_node.hpp"


namespace hpe_test {

    class ResizerNode : public rclcpp::Node {
        public:
            ResizerNode(std::string name, std::string raw_topic, int h, int w, int starting_workers);
            ~ResizerNode();

        private:
            float computeIoU(const float* box1, const float* box2);
            std::vector<size_t> nonMaxSuppression(float* boxes, float* scores, size_t numBoxes, float iouThreshold, float minConfidence);
            void create_new_worker();
            void setupTensors();
            void callback(const sensor_msgs::msg::Image &msg);
            void open_camera();
            void findPpl(cv::Mat&, std_msgs::msg::Header);


            std::string model_file 			= "./src/hpe_test/models/4.tflite";
            std::string boxes_model_file 	= "./src/hpe_test/detector_models/yolov4-tiny-416.tflite";
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_boxes_;
            std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
            int width = 192;
            int height = 192;
            std::unique_ptr<tflite::FlatBufferModel> model;
            std::unique_ptr<tflite::Interpreter> interpreter;
            int input_width = 416;
            int input_height = 416;
            int input_tensor_idx;
            int output_tensor_idx;
            int workers_n = 0;
            TfLiteType inputType;
            TfLiteType outputType;
            int input_size;
            int output_size;
            TfLiteIntArray *input_dims;
            TfLiteIntArray *output_dims;
            std::string camera_frame;
            cv_bridge::CvImage cv_image_msg_bridge;
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat small;
            cv::VideoCapture cap;
            sensor_msgs::msg::Image small_msg;
            float *input_data;
            float *output_data;
            std::string node_name;
            std::vector<std::thread> worker_threads;
        };

} 
#endif