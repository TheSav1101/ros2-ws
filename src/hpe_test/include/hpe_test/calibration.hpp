#ifndef HPE_TEST__CALIBRATION_HPP_
#define HPE_TEST__CALIBRATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include "hpe_msgs/srv/calibration.hpp"
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace hpe_test{

    class Calibration {
    public:
        Calibration(hpe_msgs::msg::Calibration &calibration_msg);
        ~Calibration();

        const Eigen::Matrix4f &             getExtrinsics() const;
        const Eigen::Matrix<float, 3, 4> &  getIntrinsics() const;
        const Eigen::Matrix<float, 3, 4> &  getProjection() const;
        const Eigen::Matrix<float, 3, 4> &  getM()          const; //same as getProjection

        Eigen::Matrix<float, 2, 3>          getARows(Eigen::Vector2f p);
        Eigen::Matrix<float, 2, 1>          getBRows(Eigen::Vector2f p);
        

    private:
        Eigen::Matrix4f             extrinsics_;    //wT
        Eigen::Matrix<float, 3, 4>  intrinsics_;    //I
        Eigen::Matrix<float, 3, 4>  projection_;    //M
    };
}


#endif