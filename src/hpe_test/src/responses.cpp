#include <hpe_test/responses.hpp>

namespace hpe_test{

    Responses::Responses(int size){
        size_ = size;
        responses = {};
    }

    Responses::~Responses(){}

    bool Responses::isReady(){
        return (int)responses.size() == size_;
    }

    void Responses::add(hpe_msgs::msg::Hpe2d& msg){
        if((int)responses.size() < size_){
            responses.push_back(msg);
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OUT OF BOUNDS");
        }
    }

    std::vector<hpe_msgs::msg::Hpe2d> Responses::get(){
        return responses;
    }
}