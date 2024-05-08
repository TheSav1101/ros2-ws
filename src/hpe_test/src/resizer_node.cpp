#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <cv_bridge/cv_bridge.h>

#include "hpe_msgs/msg/hpe2d.hpp"

using std::placeholders::_1;
// const std::string model_file = "./models/movenet-singlepose-lightning.tflite";
const std::string model_file = "./models/4.tflite";
class ResizerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int width = 192;
    int height = 192;

    std::string camera_frame;

    cv_bridge::CvImage cv_image_msg_bridge;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat small;
    cv::VideoCapture cap;

    sensor_msgs::msg::Image small_msg;

    void callback(const sensor_msgs::msg::Image &msg)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {

            RCLCPP_ERROR(this->get_logger(), "ERROR: cv_bridge exception %s", e.what());
        }

        cv::resize(cv_ptr->image, small, cv::Size(width, height), cv::INTER_LINEAR);

        cv_image_msg_bridge = cv_bridge::CvImage(msg.header, sensor_msgs::image_encodings::BGR8, small);
        cv_image_msg_bridge.toImageMsg(small_msg);
        publisher_->publish(small_msg);
    }

    void open_camera()
    {
        cap = cv::VideoCapture(0);

        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: no video feed...");
        }

        while (1)
        {
            cv::Mat frame;
            cap >> frame;

            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR: missin a frame...");
            }
            cv::resize(cv_ptr->image, small, cv::Size(width, height), cv::INTER_LINEAR);

            std_msgs::msg::Header header;
            header.set__stamp(this->get_clock()->now());
            header.set__frame_id(camera_frame);
            cv_image_msg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, small);
            cv_image_msg_bridge.toImageMsg(small_msg);
            publisher_->publish(small_msg);

            if (cv::waitKey(1) == 'q')
            {
                break;
            }
        }

        cap.release();
    }

public:
    ResizerNode(std::string name, std::string raw_topic, int h, int w) : Node(name)
    {
        width = w;
        height = h;
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/resized/" + name, 10);
        camera_frame = name;
        if (raw_topic != "null")
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                raw_topic, 10, std::bind(&ResizerNode::callback, this, _1));
        else
        {
            open_camera();
        }
        RCLCPP_INFO(this->get_logger(), "Ready");
    }
};

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        std::cout << "Usage: ros2 run hpe_test image_resizer <name> <raw_topic> <height> <width>\n";
        return 1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResizerNode>(argv[1], argv[2], atoi(argv[3]), atoi(argv[4])));
    rclcpp::shutdown();
    return 0;
}