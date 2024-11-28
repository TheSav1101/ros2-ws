#ifndef HPE_TEST__RESPONSES_HPP_
#define HPE_TEST__RESPONSES_HPP_

#include <hpe_msgs/srv/estimate.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hpe_test {

class Responses {
public:
  Responses(const int size);
  ~Responses();
  bool isReady();
  bool add(const hpe_msgs::msg::Hpe2d &msg);
  std::vector<hpe_msgs::msg::Hpe2d> get();

private:
  std::vector<hpe_msgs::msg::Hpe2d> responses;
  int size_;

  // sinceramente spero non serva ma non si sa mai
  std::mutex mutex_;
};
} // namespace hpe_test

#endif
