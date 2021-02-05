#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <popl.hpp>
#include <string>
#include <thread>

#include "System.h"
#include "ImuTypes.h"

#include "cameras/l515.h"

void tracking(const std::string& vocab_file_path, const std::string& config_file_path,
              const std::string& logdir) {
  L515Async camera;

  ORB_SLAM3::System SLAM(vocab_file_path, config_file_path,
                         ORB_SLAM3::System::IMU_MONOCULAR, true);
  while (true) {
    const auto data = camera.GetSyncedData();
    std::vector<ORB_SLAM3::IMU::Point> imu_sequence;
    //std::cout << "PACKET @ " << data.timestamp << std::endl;
    for (const auto& imu_frame: data.imu_frames) {
      const ORB_SLAM3::IMU::Point p(imu_frame.accel[0], imu_frame.accel[1], imu_frame.accel[2],
                                    imu_frame.gyro[0], imu_frame.gyro[1], imu_frame.gyro[2],
                                    imu_frame.timestamp);
      //std::cout << "  IMU @ " << imu_frame.timestamp << ' ';
      //std::cout << "Accel: " << p.a << ' ';
      //std::cout << "Gyro: " << p.w << std::endl;
      imu_sequence.push_back(p);
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //std::cout << std::endl;

    const auto st = std::chrono::steady_clock::now();
    cv::Mat rgb_small;
    cv::resize(data.rgb_img, rgb_small, cv::Size(), .5, .5);
    SLAM.TrackMonocular(rgb_small, data.timestamp, imu_sequence);
    const auto end = std::chrono::steady_clock::now();
    //std::cout << "Taked " << std::chrono::duration<double>(end - st).count() << " s" << std::endl;
  }

  SLAM.Shutdown();
}

int main(int argc, char* argv[]) {
  popl::OptionParser op("Allowed options");
  auto help = op.add<popl::Switch>("h", "help", "produce help message");
  auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
  auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
  auto log_dir =
      op.add<popl::Value<std::string>>("", "logdir", "directory to store logged data", "./log");
  try {
    op.parse(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << std::endl;
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  if (help->is_set()) {
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  if (!vocab_file_path->is_set() || !config_file_path->is_set()) {
    std::cerr << "Invalid Arguments" << std::endl;
    std::cerr << std::endl;
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  ORB_SLAM3::Verbose::th = ORB_SLAM3::Verbose::VERBOSITY_DEBUG;

  tracking(vocab_file_path->value(), config_file_path->value(), log_dir->value());

  return EXIT_SUCCESS;
}
