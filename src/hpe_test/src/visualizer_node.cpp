#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "hpe_msgs/msg/hpe2d.hpp"
#include <deque>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

using std::placeholders::_1;

struct Keypoint
{
    float x;
    float y;
};

struct Keypoint_score
{
    float x;
    float y;
    float score;
};

class visualizerNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<hpe_msgs::msg::Hpe2d>::SharedPtr subscription_hpe;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image;
    sensor_msgs::msg::Image image;

    void callback2d(hpe_msgs::msg::Hpe2d hpe_result)
    {
        RCLCPP_INFO(this->get_logger(), "Working on a new image...");

        std::vector<Keypoint_score> keypoints_score = keypoints_scores_from_msgs(hpe_result);

        std::vector<Keypoint> kp_viz = keypoints_and_edges_for_display(keypoints_score, image.height, image.width);

        try
        {
            cv_bridge::CvImagePtr cv_ptr;

            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

            cv::Mat image_cv = cv_ptr->image;

            for (size_t i = 0; i < kp_viz.size(); i++)
            {
                cv::circle(image_cv, cv::Point(kp_viz[i].x, kp_viz[i].y), 10, cv::Scalar(0, 255, 0));
            }

            cv_bridge::CvImage cv_image_msg_bridge;
            cv_image_msg_bridge = cv_bridge::CvImage(image.header, sensor_msgs::image_encodings::BGR8, image_cv);
            sensor_msgs::msg::Image output;
            cv_image_msg_bridge.toImageMsg(output);
            publisher_->publish(output);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("cv_bridge"), "cv_bridge exception: " << e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Fnished visualizing the image...");
    }

    void image_callback(sensor_msgs::msg::Image msg)
    {
        image = msg;
    }

    std::vector<Keypoint_score> keypoints_scores_from_msgs(hpe_msgs::msg::Hpe2d msg)
    {
        std::vector<Keypoint_score> kpts = {};
        for (int i = 0; i < msg.dim; i++)
        {
            Keypoint_score kpt{msg.joints_x[i], msg.joints_y[i], msg.confidence[i]};
            kpts.push_back(kpt);
        }

        return kpts;
    }

    std::vector<Keypoint> keypoints_and_edges_for_display(
        const std::vector<Keypoint_score> &keypoints_with_scores,
        int height,
        int width,
        float keypoint_threshold = 0.1)
    {

        std::vector<Keypoint> keypoints_all;

        for (size_t i = 0; i < keypoints_with_scores.size(); ++i)
        {
            float abs_x = keypoints_with_scores[i].x * width;
            float abs_y = keypoints_with_scores[i].y * height;

            if (keypoints_with_scores[i].score > keypoint_threshold)
            {
                Keypoint keypoint;
                keypoint.x = abs_x;
                keypoint.y = abs_y;
                keypoints_all.push_back(keypoint);
            }
        }

        return keypoints_all;
    }

public:
    visualizerNode() : Node("visualizer")
    {

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hpe_visual", 42);

        subscription_hpe = this->create_subscription<hpe_msgs::msg::Hpe2d>(
            "/hpe_result", 1, std::bind(&visualizerNode::callback2d, this, _1));

        subscription_image = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", 1, std::bind(&visualizerNode::image_callback, this, _1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<visualizerNode>());
    rclcpp::shutdown();
    return 0;
}