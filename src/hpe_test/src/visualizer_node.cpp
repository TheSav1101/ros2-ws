#include <hpe_test/visualizer_node.hpp>

using std::placeholders::_1;

namespace hpe_test{
    void VisualizerNode::callback2d(const std::shared_ptr<hpe_msgs::msg::Slave> hpe_result)
    {
        RCLCPP_INFO(this->get_logger(), "Working on a new image...");

        std::vector<Keypoint_score> keypoints_score = keypoints_scores_from_msgs(hpe_result);

        std::vector<Keypoint> kp_viz = keypoints_and_edges_for_display(keypoints_score, image.height, image.width, 0.4);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

            image_cv = cv_ptr->image;

            for (size_t i = 0; i < kp_viz.size(); i++)
            {
                cv::circle(image_cv, cv::Point(kp_viz[i].x, kp_viz[i].y), 4, cv::Scalar(255, 0, 255));
            }

            cv_image_msg_bridge = cv_bridge::CvImage(image.header, sensor_msgs::image_encodings::BGR8, image_cv);

            cv_image_msg_bridge.toImageMsg(output);
            publisher_->publish(output);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("cv_bridge"), "cv_bridge exception: " << e.what());
        }

        double delay = (this->get_clock()->now() - hpe_result->header.stamp).seconds();

        avg_delay = (avg_delay*delay_window + delay);
        delay_window++;
        avg_delay /= delay_window;
        RCLCPP_INFO(this->get_logger(), "Average delay: %f, window: %d", avg_delay, delay_window);
    }

    void VisualizerNode::image_callback(sensor_msgs::msg::Image msg)
    {
        image = msg;
    }

    std::vector<Keypoint_score> VisualizerNode::keypoints_scores_from_msgs(const std::shared_ptr<hpe_msgs::msg::Slave> msg)
    {
        std::vector<Keypoint_score> kpts = {};
        for (int i = 0; i < msg->skeletons_n; i++){
            for (int j = 0; j < msg->all_joints[i].dim; j++){
                Keypoint_score kpt{msg->all_joints[i].x[j], msg->all_joints[i].y[j], msg->all_joints[i].confidence[j]};
                kpts.push_back(kpt);
            }
        }

        return kpts;
    }

    std::vector<Keypoint> VisualizerNode::keypoints_and_edges_for_display(const std::vector<Keypoint_score> &keypoints_with_scores, int height, int width, float keypoint_threshold){

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

    VisualizerNode::~VisualizerNode(){}

    VisualizerNode::VisualizerNode(std::string name) : Node("visualizer_" + name){
        delay_window = 0;
        avg_delay = 0.0;
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/hpe_visual/" + name, 10);

        subscription_slave_ = this->create_subscription<hpe_msgs::msg::Slave>(
            "/slave_" + name, 10, std::bind(&VisualizerNode::callback2d, this, _1));

        subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/boxes_" + name, 10, std::bind(&VisualizerNode::image_callback, this, _1));
    }
};
