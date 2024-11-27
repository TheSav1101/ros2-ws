#include <hpe_test/calibration.hpp>

namespace hpe_test {
Calibration::Calibration(hpe_msgs::msg::Calibration &calibration_msg) {

  extrinsics_ = Eigen::Matrix4f::Identity();

  float tx = calibration_msg.frame.transform.translation.x;
  float ty = calibration_msg.frame.transform.translation.y;
  float tz = calibration_msg.frame.transform.translation.z;

  extrinsics_(0, 3) = tx;
  extrinsics_(1, 3) = ty;
  extrinsics_(2, 3) = tz;

  float qx = calibration_msg.frame.transform.rotation.x;
  float qy = calibration_msg.frame.transform.rotation.y;
  float qz = calibration_msg.frame.transform.rotation.z;
  float qw = calibration_msg.frame.transform.rotation.w;

  tf2::Quaternion quaternion(qx, qy, qz, qw);
  quaternion.normalize();
  tf2::Matrix3x3 rotation_matrix(quaternion);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      extrinsics_(i, j) = rotation_matrix[i][j];
    }
  }

  // Fine extrinsics

  Eigen::Matrix3f partial_intrinsics = Eigen::Matrix3f::Identity();
  for (int col = 0; col < partial_intrinsics.cols(); ++col) {
    for (int row = 0; row < partial_intrinsics.rows(); ++row) {
      partial_intrinsics(row, col) =
          calibration_msg.intrinsic_params
              .camera_matrix[col * partial_intrinsics.rows() + row];
    }
  }

  std::cout << "Intrinsic matrix:\n" << partial_intrinsics << std::endl;

  intrinsics_ = Eigen::Matrix<float, 3, 4>::Zero();
  intrinsics_.block<3, 3>(0, 0) = partial_intrinsics;

  // Fine intrinsics

  Eigen::Matrix4f extrinsics_inv = extrinsics_.inverse();
  projection_ = intrinsics_ * extrinsics_inv;

  std::cout << "Projection matrix:\n" << projection_ << std::endl;
}

Calibration::~Calibration() {}

Calibration::Calibration() {}

const Eigen::Matrix4f &Calibration::getExtrinsics() const {
  return extrinsics_;
};
const Eigen::Matrix<float, 3, 4> &Calibration::getIntrinsics() const {
  return intrinsics_;
};
const Eigen::Matrix<float, 3, 4> &Calibration::getProjection() const {
  return projection_;
};
const Eigen::Matrix<float, 3, 4> &Calibration::getM() const {
  return projection_;
};

Eigen::Matrix<float, 2, 3> Calibration::getARows(Eigen::Vector2f p) {
  Eigen::Matrix<float, 2, 3> out = Eigen::Matrix<float, 2, 3>::Zero();

  out(0, 0) = projection_(2, 0) * p(0) - projection_(0, 0);
  out(0, 1) = projection_(2, 1) * p(0) - projection_(0, 1);
  out(0, 2) = projection_(2, 2) * p(0) - projection_(0, 2);

  out(1, 0) = projection_(2, 0) * p(1) - projection_(1, 0);
  out(1, 1) = projection_(2, 1) * p(1) - projection_(1, 1);
  out(1, 2) = projection_(2, 2) * p(1) - projection_(1, 2);

  return out;
};

Eigen::Matrix<float, 2, 1> Calibration::getBRows(Eigen::Vector2f p) {
  Eigen::Matrix<float, 2, 1> out = Eigen::Matrix<float, 2, 1>::Zero();

  out(0, 0) = projection_(0, 3) - p(0) * projection_(2, 3);
  out(1, 0) = projection_(1, 3) - p(1) * projection_(2, 3);

  return out;
};

} // namespace hpe_test
