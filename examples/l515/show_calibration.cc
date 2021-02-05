#include <iostream>

#include "cameras/l515.h"

void PrintVideoIntrinsics(const rs2_intrinsics& intrinsics) {
  std::cout << "Focal: " << intrinsics.fx << ' ' << intrinsics.fy << std::endl;
  std::cout << "Principle: " << intrinsics.ppx << ' ' << intrinsics.ppy << std::endl;
  std::cout << "Distortion Type: " << intrinsics.model << std::endl;
  std::cout << "Distortion Coeff: ";
  for (int i = 0; i < 5; ++i) {
    std::cout << intrinsics.coeffs[i] << ' ';
  }
  std::cout << std::endl << std::endl;
}

void PrintMotionIntrinsics(const rs2_motion_device_intrinsic& intrinsics) {
  const Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> data((float*)intrinsics.data);
  std::cout << "Intrinsics: " << std::endl << data << std::endl;

  std::cout << "Noise Variance: ";
  for (int i = 0; i < 3; ++i) { std::cout << intrinsics.noise_variances[i] << ' '; }
  std::cout << std::endl;

  std::cout << "Bias Variance: ";
  for (int i = 0; i < 3; ++i) { std::cout << intrinsics.bias_variances[i] << ' '; }
  std::cout << std::endl << std::endl;
}

void PrintExtrinsics(const rs2_extrinsics& extrinsics) {
  const Eigen::Map<const Eigen::Matrix3f, Eigen::RowMajor> R(extrinsics.rotation);
  const Eigen::Map<const Eigen::Vector3f> t(extrinsics.translation);
  std::cout << "R:\n" << R << std::endl;
  std::cout << "t: " << t.transpose() << std::endl;
}

int main() {
  L515Async l515(false);

  std::cout << "[RGB Intrinsics]" << std::endl;
  PrintVideoIntrinsics(l515.GetVideoIntrinsics<RS2_STREAM_COLOR>());

  std::cout << "[Depth Intrinsics]" << std::endl;
  PrintVideoIntrinsics(l515.GetVideoIntrinsics<RS2_STREAM_DEPTH>());

  std::cout << "[Accel Intrinsics]" << std::endl;
  PrintMotionIntrinsics(l515.GetMotionIntrincis<RS2_STREAM_ACCEL>());

  std::cout << "[Gyro Intrinsics]" << std::endl;
  PrintMotionIntrinsics(l515.GetMotionIntrincis<RS2_STREAM_GYRO>());

  std::cout << "[IMU_T_RGB]" << std::endl;
  PrintExtrinsics(l515.GetExtrinsics<RS2_STREAM_ACCEL, RS2_STREAM_COLOR>());

  return 0;
}
