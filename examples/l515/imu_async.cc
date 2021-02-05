#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>

#include <librealsense2/rs.hpp>

#include "cameras/l515.h"

int main() {
  rs2::config cfg;
  cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
  cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, rs2_format::RS2_FORMAT_Z16, 30);
  cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);
  cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);
  rs2::pipeline pipe;

  rs2::frameset fss;
  if (fss) {
    std::cout << "asdf\nasldf\nasldfj\nasdfljk\n";
  }

  ImuSyncer syncer([] (const imu_frame_t& imu_frame) {
    printf("[ASYNC] @ %.2f ms ", imu_frame.timestamp * 1e3);
    std::cout << "GYRO: " << imu_frame.gyro.transpose() << " ACCEL: " << imu_frame.accel.transpose() << std::endl;
  });

  pipe.start(cfg, [&](rs2::frame frame) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      printf("[SYNC] @ %.2f ms\n", fs.get_timestamp());
      for (const rs2::frame& f: fs) {
        printf("%s\n", f.get_profile().stream_name().c_str());
      }
    } else {
      syncer.AddMeasurement(frame);
    }
  });

  while (true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }

  return 0;
}
