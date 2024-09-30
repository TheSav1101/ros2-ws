#ifndef HPE_TEST__VISUALIZER_NODE_HPP_
#define HPE_TEST__VISUALIZER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "hpe_msgs/srv/estimate.hpp"
#include "hpe_msgs/msg/slave.hpp"
#include "hpe_test/data.hpp"
#include <deque>



namespace hpe_test {

    struct Keypoint{
        float x;
        float y;
    };

    struct Keypoint_score{
        float x;
        float y;
        float score;
    };

    class VisualizerNode : public rclcpp::Node {
        public:
            VisualizerNode(std::string name);
            ~VisualizerNode();
            void callback2d(const std::shared_ptr<hpe_msgs::msg::Slave> hpe_result);
            void image_callback(sensor_msgs::msg::Image msg);
            std::vector<Keypoint> keypoints_and_edges_for_display( const std::vector<Keypoint_score> &keypoints_with_scores, int height, int width, float keypoint_threshold);
            std::vector<Keypoint_score> keypoints_scores_from_msgs(const std::shared_ptr<hpe_msgs::msg::Slave> msg);

        private:
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
            rclcpp::Subscription<hpe_msgs::msg::Slave>::SharedPtr subscription_slave_;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
            sensor_msgs::msg::Image image;
            cv_bridge::CvImagePtr cv_ptr;
            cv_bridge::CvImage cv_image_msg_bridge;
            cv::Mat image_cv;
            sensor_msgs::msg::Image output;
            double avg_delay;
            int delay_window;
        };

} 
#endif