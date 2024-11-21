#ifndef HPE_TEST__WORKER_NODE_HPP_
#define HPE_TEST__WORKER_NODE_HPP_

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include "hpe_msgs/srv/estimate.hpp"
#include "hpe_test/data.hpp"

namespace hpe_test {

    class WorkerNode : public rclcpp::Node {
        public:
            WorkerNode(std::string name, int model);
            ~WorkerNode();
            void shutdown();
            bool isReady();

        private:
            void setupTensors();
            void service(const std::shared_ptr<hpe_msgs::srv::Estimate::Request> request, std::shared_ptr<hpe_msgs::srv::Estimate::Response> response);

            int hpe_model_n = 0;

            rclcpp::Service<hpe_msgs::srv::Estimate>::SharedPtr service_;

            std::atomic<bool> occupied_;
            
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

            int delay_window;
            double avg_delay;
            
            hpe_msgs::msg::Hpe2d hpe_msg;
        };

} 
#endif