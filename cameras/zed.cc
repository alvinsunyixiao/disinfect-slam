#include "zed.h"

ZED::ZED() {
  sl::InitParameters init_params;
  init_params.camera_resolution = sl::RESOLUTION::VGA;
  init_params.camera_fps = 60;
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
  zed_.open(init_params);
  zed_.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, 100);
  rt_params_ = zed_.getRuntimeParameters();
  rt_params_.confidence_threshold = 50;
  cam_config_ = zed_.getCameraInformation().camera_configuration;
  sensor_config_ = zed_.getCameraInformation().sensors_configuration;

  // start position tracking
  zed_.enablePositionalTracking();
}

ZED::~ZED() { zed_.close(); }

sl::CameraConfiguration ZED::GetCameraConfig() const { return cam_config_; }

sl::SensorsConfiguration ZED::GetSensorConfig() const { return sensor_config_; }

void ZED::GetStereoAndRGBDFrame(cv::Mat* left_img, cv::Mat* right_img, cv::Mat* rgb_img,
                                cv::Mat* depth_img) {
  AllocateIfNeeded(left_img, CV_8UC1);
  AllocateIfNeeded(right_img, CV_8UC1);
  AllocateIfNeeded(rgb_img, CV_8UC4);
  AllocateIfNeeded(depth_img, CV_32FC1);

  sl::Mat left_sl(cam_config_.resolution, sl::MAT_TYPE::U8_C1, left_img->data,
                  cam_config_.resolution.width);
  sl::Mat right_sl(cam_config_.resolution, sl::MAT_TYPE::U8_C1, right_img->data,
                   cam_config_.resolution.width);
  sl::Mat rgb_sl(cam_config_.resolution, sl::MAT_TYPE::U8_C4, rgb_img->data,
                 cam_config_.resolution.width * 4);
  sl::Mat depth_sl(cam_config_.resolution, sl::MAT_TYPE::F32_C1, depth_img->data,
                   cam_config_.resolution.width * sizeof(float));

  if (zed_.grab(rt_params_) == sl::ERROR_CODE::SUCCESS) {
    zed_.retrieveImage(left_sl, sl::VIEW::LEFT_GRAY);
    zed_.retrieveImage(right_sl, sl::VIEW::RIGHT_GRAY);
    zed_.retrieveImage(rgb_sl, sl::VIEW::LEFT);
    zed_.retrieveMeasure(depth_sl, sl::MEASURE::DEPTH);
  }
}

timed_pose_t ZED::GetTimedPose() {
  timed_pose_t ret;

  if (zed_.grab(rt_params_) == sl::ERROR_CODE::SUCCESS) {
    sl::Pose pose;
    zed_.getPosition(pose, sl::REFERENCE_FRAME::WORLD);

    const sl::Orientation R_zed = pose.getOrientation();
    const sl::Translation t_zed = pose.getTranslation();
    const Eigen::Quaternionf R(R_zed.ow, R_zed.ox, R_zed.oy, R_zed.oz);
    const Eigen::Vector3f t(t_zed.tx, t_zed.ty, t_zed.tz);

    ret.world_T_cam = SE3<float>(R, t);
    ret.timestamp = pose.timestamp.getMilliseconds();
    ret.confidence = pose.pose_confidence;
  }

  std::lock_guard<std::mutex> lock(mtx_pose_);
  world_T_cam_ = ret.world_T_cam;

  return ret;
}

SE3<float> ZED::GetWorld_T_Cam() const {
  std::lock_guard<std::mutex> lock(mtx_pose_);
  return world_T_cam_;
}

void ZED::AllocateIfNeeded(cv::Mat* img, int type) const {
  if (img->empty() || img->type() != type || img->cols != cam_config_.resolution.width ||
      img->rows != cam_config_.resolution.height)
    *img = cv::Mat(cam_config_.resolution.height, cam_config_.resolution.width, type);
}
