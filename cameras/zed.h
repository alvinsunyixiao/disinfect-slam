#pragma once

#include <mutex>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "utils/cuda/lie_group.cuh"
#include "utils/rotation_math/pose_manager.h"

struct timed_pose_t {
  SE3<float> world_T_cam;
  int confidence;
  uint64_t timestamp;

  timed_pose_t() : world_T_cam(SE3<float>::Identity()), confidence(0), timestamp(0) {};
};

/**
 * @brief ZED camera interface using ZED SDK
 */
class ZED {
 public:
  ZED();
  ~ZED();

  /**
   * @return camera config including image specs and calibration parameters
   */
  sl::CameraConfiguration GetConfig() const;

  /**
   * @brief read both stereo and RGBD frames
   *
   * @param left_img    left image of stereo frame
   * @param right_img   right image of stereo frame
   * @param rgb_img     rgb image of RGBD frame
   * @param depth_img   depth image of RGBD frame
   */
  void GetStereoAndRGBDFrame(cv::Mat* left_img, cv::Mat* right_img, cv::Mat* rgb_img,
                             cv::Mat* depth_img);

  timed_pose_t GetTimedPose();

  SE3<float> GetWorld_T_Cam() const;

 private:
  void AllocateIfNeeded(cv::Mat* img, int type) const;

  sl::Camera zed_;
  sl::CameraConfiguration config_;
  sl::RuntimeParameters rt_params_;

  mutable std::mutex mtx_pose_;
  SE3<float> world_T_cam_;
};
