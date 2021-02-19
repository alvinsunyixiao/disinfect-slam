#include <chrono>
#include <iostream>

#include "cameras/zed.h"
#include "utils/time.hpp"

int main(int argc, char* argv[]) {
  ZED camera;

  while (true) {
    auto pose = camera.GetTimedPose();
    std::cout << "Packet Time: " << pose.timestamp << " ms ";
    std::cout << "Unix Time: " << GetSystemTimestamp<std::chrono::milliseconds>() << " ms" << std::endl;
    std::cout << pose.world_T_cam.GetT() << std::endl;
  }

  return EXIT_SUCCESS;
}
