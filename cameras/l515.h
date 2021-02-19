#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

struct timed_rgbd_frame_t {
  cv::Mat color_img;
  cv::Mat depth_img;
  uint64_t timestamp;
};

/**
 * @brief L515 camer interface with librealsense2
 */
class L515 {
 public:
  L515();
  ~L515();

  /**
   * @brief read an RGBD frame
   *
   * @param color_img rgb image
   * @param depth_img depth image
   *
   * @return timestamp in system clock
   */
  uint64_t GetRGBDFrame(cv::Mat* color_img, cv::Mat* depth_img) const;

  timed_rgbd_frame_t GetRGBDFrame() const;

  /**
   * @brief set capture properties through librealsense
   *
   * @param option  capture option
   * @param value   value to be set
   */
  void SetDepthSensorOption(const rs2_option option, const float value);

  static const int WIDTH = 1280;
  static const int HEIGHT = 720;
  static const int FPS = 30;

 private:
  rs2::config cfg_;
  rs2::pipeline pipe_;
  rs2::pipeline_profile pipe_profile_;
  rs2::align align_to_color_;
  double depth_scale_;
};
