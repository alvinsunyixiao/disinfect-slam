#include <chrono>
#include <iostream>
#include <thread>

#include "cameras/l515.h"

int main() {
  L515Async l515;

  while (true) {
    const l515_data_t data = l515.GetSyncedData();
    std::cout << "[Packet @ " << data.timestamp << "]" << std::endl;
    std::cout << "  Image size: " << data.depth_img.size() << std::endl;
    for (const auto& imu_frame: data.imu_frames) {
      std::cout << "    IMU @ " << imu_frame.timestamp << std::endl;
    }

    cv::Mat bgr;
    cv::cvtColor(data.rgb_img, bgr, cv::COLOR_RGB2BGR);
    cv::imshow("rgb", bgr);
    cv::imshow("depth", data.depth_img);

    cv::waitKey(10);
  }

  return 0;
}
