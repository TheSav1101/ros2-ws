#ifndef HPE_TEST__RESPONSES_HPP_
#define HPE_TEST__RESPONSES_HPP_

#include <rclcpp/rclcpp.hpp>
#include "hpe_msgs/srv/estimate.hpp"

namespace hpe_test{

    class Responses {
    public:
        Responses(int size);
        ~Responses();
        bool isReady();
        void add(hpe_msgs::msg::Hpe2d& msg);        
        std::vector<hpe_msgs::msg::Hpe2d> get();
    private:
        std::vector<hpe_msgs::msg::Hpe2d> responses;
        int size_;

    };
}


#endif