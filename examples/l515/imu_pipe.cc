#include <cstdio>
#include <librealsense2/rs.hpp>

int main() {
  rs2::config cfg;
  cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);
  cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);

  rs2::pipeline pipe;
  pipe.start(cfg);

  while (true) {
    rs2::frameset fs = pipe.wait_for_frames();

    printf("[Packet @ %.2f ms] ", fs.get_timestamp());
    for (const rs2::frame& f: fs) {
      printf("%s @ %.2f ms ", f.get_profile().stream_name().c_str(), f.get_timestamp());
    }
    printf("\n");
  }
}
