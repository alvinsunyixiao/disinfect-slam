#include <cstdio>
#include <librealsense2/rs.hpp>

int main() {
  rs2::config cfg;
  cfg.enable_stream(rs2_stream::RS2_STREAM_ACCEL, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);
  cfg.enable_stream(rs2_stream::RS2_STREAM_GYRO, rs2_format::RS2_FORMAT_MOTION_XYZ32F, 200);
  rs2::pipeline pipe;

  pipe.start(cfg, [](rs2::frame frame) {
    printf("%s @ %.10f ms @ %d \n", frame.get_profile().stream_name().c_str(), frame.get_timestamp(), frame.get_frame_timestamp_domain());
  });

  while (true);

  return 0;
}
