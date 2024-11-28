#include <hpe_test/responses.hpp>

namespace hpe_test {

Responses::Responses(const int size) {
  size_ = size;
  responses = {};
}

Responses::~Responses() {}

bool Responses::isReady() { return (int)responses.size() == size_; }

bool Responses::add(const hpe_msgs::msg::Hpe2d &msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((int)responses.size() < size_) {
      responses.push_back(msg);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "OUT OF BOUNDS");
    }
    return (int)responses.size() == size_;
  }
}

std::vector<hpe_msgs::msg::Hpe2d> Responses::get() { return responses; }
} // namespace hpe_test
