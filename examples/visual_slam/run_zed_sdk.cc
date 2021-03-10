#include <chrono>
#include <iostream>

#include "cameras/zed.h"
#include "utils/time.hpp"

int main(int argc, char* argv[]) {
  ZED camera;

  const auto sensor_cfg = camera.GetSensorConfig();
  std::cout << "Accel Noise: " << sensor_cfg.accelerometer_parameters.noise_density << std::endl;
  std::cout << "Accel Walk: " << sensor_cfg.accelerometer_parameters.random_walk << std::endl;
  std::cout << "Gyro Noise: " << sensor_cfg.gyroscope_parameters.noise_density << std::endl;
  std::cout << "Gyro Walk: " << sensor_cfg.gyroscope_parameters.random_walk << std::endl;
  std::cout << "Accel Sampling Rate: " << sensor_cfg.accelerometer_parameters.sampling_rate << std::endl;
  std::cout << "Gyro Sampling Rate: " << sensor_cfg.gyroscope_parameters.sampling_rate << std::endl;

  const auto camera_cfg = camera.GetCameraConfig();
  std::cout << "Camera Params: " << std::endl;
  std::cout << "  fx: " << camera_cfg.calibration_parameters.left_cam.fx << std::endl;
  std::cout << "  fy: " << camera_cfg.calibration_parameters.left_cam.fy << std::endl;
  std::cout << "  cx: " << camera_cfg.calibration_parameters.left_cam.cx << std::endl;
  std::cout << "  cy: " << camera_cfg.calibration_parameters.left_cam.cy << std::endl;

  while (true) {
    auto pose = camera.GetTimedPose();
    std::cout << "Packet Time: " << pose.timestamp << " ms ";
    std::cout << "Unix Time: " << GetSystemTimestamp<std::chrono::milliseconds>() << " ms" << std::endl;
    std::cout << pose.world_T_cam.GetT() << std::endl;
  }

  return EXIT_SUCCESS;
}
