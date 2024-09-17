#ifndef HPE_TEST__WORKER_NODE_HPP_
#define HPE_TEST__WORKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include "hpe_msgs/msg/hpe2d.hpp"
#include "hpe_msgs/msg/detection.hpp"
#include "hpe_msgs/msg/box.hpp"
#include "hpe_test/data.inl"


namespace hpe_test {

    class WorkerNode : public rclcpp::Node {
        public:
            WorkerNode(std::string name, int model);
            ~WorkerNode();

        private:
            void setupTensors();
            void callback(const hpe_msgs::msg::Detection &msg);

            int hpe_model_n = 0;

            rclcpp::Publisher<hpe_msgs::msg::Hpe2d>::SharedPtr publisher_;
            rclcpp::Subscription<hpe_msgs::msg::Detection>::SharedPtr subscription_;
            
            std::unique_ptr<tflite::FlatBufferModel> model;
            std::unique_ptr<tflite::Interpreter> interpreter;
            
            int input_tensor_idx;
            int output_tensor_idx;
            TfLiteType inputType;
            TfLiteType outputType;
            int input_size;
            int output_size;
            TfLiteIntArray *input_dims;
            TfLiteIntArray *output_dims;
            cv_bridge::CvImagePtr cv_ptr;
            
            uint8_t *input_data;
            float *output_data;

            hpe_msgs::msg::Hpe2d hpe_msg;
        };

} 
#endif