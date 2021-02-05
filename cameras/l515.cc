#include "l515.h"

#include <iterator>

//#include <spdlog/spdlog.h>

#define RGB_WIDTH 1280
#define RGB_HEIGHT 720
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

#define IMG_FPS 30
#define IMU_FPS 200

#define MILLI2SEC(ms) ((ms) / 1e3)

ImuSyncer::ImuSyncer(const CallbackFunc& callback) : callback_(callback) {}

void ImuSyncer::AddMeasurement(const rs2::frame& frame) const {
  const rs2::motion_frame m_frame = frame.as<rs2::motion_frame>();
  if (!m_frame) {
    //spdlog::error("IMU Syncer received non IMU data");
    return;
  }

  if (m_frame.get_profile().stream_type() == rs2_stream::RS2_STREAM_ACCEL) {
    accel_q_.push(m_frame);
  } else if (m_frame.get_profile().stream_type() == rs2_stream::RS2_STREAM_GYRO) {
    gyro_q_.push_back(m_frame);
  } else {
    //spdlog::error("Unsupported stream: {}", m_frame.get_profile().stream_name());
  }

  TryInvokeSync();
}

void ImuSyncer::TryInvokeSync() const {
  if (accel_q_.empty() || gyro_q_.empty()) { return; }

  if (gyro_q_.front().get_timestamp() < accel_q_.back().get_timestamp() &&
      gyro_q_.back().get_timestamp() >= accel_q_.front().get_timestamp()) {
    // remove invalid data samples
    while (accel_q_.front().get_timestamp() < gyro_q_.front().get_timestamp()) {
      accel_q_.pop();
    }

    const rs2::motion_frame& accel_f = accel_q_.front();
    const double timestamp_ms = accel_f.get_timestamp();

    auto it_curr = gyro_q_.begin();
    auto it_next = std::next(it_curr, 1);

    // find zero crossing
    while (it_next != gyro_q_.end() && it_next->get_timestamp() < accel_f.get_timestamp()) {
      it_curr = (it_next++);
    }

    // synchronize and invoke callback
    if (it_next != gyro_q_.end()) {
      const double duration_ms = it_next->get_timestamp() - it_curr->get_timestamp();
      const double alpha = (it_next->get_timestamp() - timestamp_ms) / duration_ms;

      // interpolate gyro
      const auto gyro_start = Eigen::Map<Eigen::Vector3f>((float*)it_curr->get_data());
      const auto gyro_end = Eigen::Map<Eigen::Vector3f>((float*)it_next->get_data());
      const Eigen::Vector3f gyro_interp = gyro_start * alpha + gyro_end * (1 - alpha);

      // invoke callback
      const auto accel = Eigen::Map<Eigen::Vector3f>((float*)accel_f.get_data());
      callback_({
        .gyro = gyro_interp.cast<double>(),
        .accel = accel.cast<double>(),
        .timestamp = MILLI2SEC(timestamp_ms),
      });

      // clean up history
      while (!gyro_q_.empty() && gyro_q_.front().get_timestamp() <= timestamp_ms) {
        gyro_q_.pop_front();
      }
      while (!accel_q_.empty() && accel_q_.front().get_timestamp() <= timestamp_ms) {
        accel_q_.pop();
      }
    }
  }
}

L515::L515() : align_to_color_(RS2_STREAM_COLOR) {
  cfg_.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, IMG_FPS);
  cfg_.enable_stream(RS2_STREAM_COLOR, RGB_WIDTH, RGB_HEIGHT, RS2_FORMAT_RGB8, IMG_FPS);
  pipe_profile_ = pipe_.start(cfg_);
}

L515::~L515() { pipe_.stop(); }

double L515::DepthScale() const {
  const auto sensor = pipe_profile_.get_device().first<rs2::depth_sensor>();
  return 1. / sensor.get_depth_scale();
}

int64_t L515::GetRGBDFrame(cv::Mat* color_img, cv::Mat* depth_img) const {
  auto frameset = pipe_.wait_for_frames();
  frameset = align_to_color_.process(frameset);

  rs2::frame color_frame = frameset.get_color_frame();
  rs2::frame depth_frame = frameset.get_depth_frame();

  const cv::Size img_size(RGB_WIDTH, RGB_HEIGHT);

  *color_img = cv::Mat(img_size, CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
  *depth_img = cv::Mat(img_size, CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

  // Metadata of depth frame and color frame is not exactly the same
  // But depth frame is used in reconstruction. So we are returning this.
  return (int64_t)(depth_frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP));
}

void L515::SetDepthSensorOption(const rs2_option option, const float value) {
  auto sensor = pipe_profile_.get_device().first<rs2::depth_sensor>();
  if (!sensor.supports(option)) {
    //spdlog::error("{} not supported", sensor.get_option_description(option));
    return;
  }
  const auto option_range = sensor.get_option_range(option);
  if (value < option_range.min || value > option_range.max) {
    //spdlog::error("value {} out of range ([{}, {}])", value, option_range.min, option_range.max);
    return;
  }
  try {
    sensor.set_option(option, value);
  } catch (const rs2::error& e) {
    //spdlog::error("Failed to set option: {}", e.what());
  }
}

L515Async::L515Async(bool start)
    : started_(start),
      align_to_color_(rs2_stream::RS2_STREAM_COLOR),
      imu_syncer_(std::bind(&L515Async::HandleImu, this, std::placeholders::_1)) {
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, IMG_FPS);
  cfg.enable_stream(RS2_STREAM_COLOR, RGB_WIDTH, RGB_HEIGHT, RS2_FORMAT_RGB8, IMG_FPS);
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, IMU_FPS);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, IMU_FPS);

  pipe_profile_ = pipe_.start(
      cfg, std::bind(&L515Async::HandleRawStream, this, std::placeholders::_1));

  const auto depth_sensor = pipe_profile_.get_device().first<rs2::depth_sensor>();
  depth_scale_ = depth_sensor.get_depth_scale();

  const auto imu = pipe_profile_.get_device().first<rs2::motion_sensor>();
  imu.set_option(rs2_option::RS2_OPTION_ENABLE_MOTION_CORRECTION, 1);
}

L515Async::~L515Async() { pipe_.stop(); }

void L515Async::Start() {
  std::lock_guard<std::mutex> lock(mtx_start_);
  started_ = true;
}

void L515Async::Stop() {
  std::lock_guard<std::mutex> lock(mtx_start_);
  started_ = false;
}

void L515Async::HandleImu(const imu_frame_t& imu_frame) const {
  frame_write_.imu_frames.push_back(imu_frame);
  TryInvokeSwap();
}

void L515Async::HandleRawStream(const rs2::frame& frame) const {
  {
    std::lock_guard<std::mutex> lock(mtx_start_);
    if (!started_) { return; }
  }

  if (const rs2::frameset fs = frame.as<rs2::frameset>()) {
    // handle RGBD
    frame_write_.image_frames = fs;
    TryInvokeSwap();
  } else {
    imu_syncer_.AddMeasurement(frame);
  }
}

void L515Async::TryInvokeSwap() const {
  if (!frame_write_.image_frames || frame_write_.imu_frames.empty()) { return; }

  // invoke swap whenever a sequence of IMU data prior to image frame is gathered
  const double img_timestamp = MILLI2SEC(frame_write_.image_frames.get_timestamp());
  if (frame_write_.imu_frames.back().timestamp > img_timestamp) {
    auto it = frame_write_.imu_frames.begin();
    while (it->timestamp < img_timestamp) { ++it; }

    // temporary buffer the imu samples for next images
    imu_sequence_t buffer;
    std::move(it, frame_write_.imu_frames.end(), std::back_inserter(buffer));
    frame_write_.imu_frames.erase(it, frame_write_.imu_frames.end());

    // swap read / write
    {
      std::lock_guard<std::mutex> lock(mtx_read_);
      // image frame dropped
      if (frame_read_.image_frames) {
        //spdlog::warn("[L515Async] image frame dropped");
        imu_sequence_t old_imu_frames = std::move(frame_read_.imu_frames);
        std::move(frame_write_.imu_frames.begin(),
                  frame_write_.imu_frames.end(),
                  std::back_inserter(old_imu_frames));
        frame_write_.imu_frames = std::move(old_imu_frames);
      }
      std::swap(frame_write_, frame_read_);
    }
    cv_read_.notify_one();

    // reset swapped in buffer
    frame_write_.image_frames = rs2::frameset();
    frame_write_.imu_frames = std::move(buffer);
  }
}

l515_data_t L515Async::GetSyncedData() const {
  rs2::frameset frameset;
  l515_data_t ret;

  // wait for a coherent new frame
  {
    std::unique_lock<std::mutex> lock(mtx_read_);
    cv_read_.wait(lock, [&]() -> bool { return frame_read_.image_frames; });

    frameset = std::move(frame_read_.image_frames);
    ret.imu_frames = std::move(frame_read_.imu_frames);
    frame_read_.image_frames = rs2::frameset();
  }

  // align depth image to color image
  const rs2::frameset aligned_frameset = align_to_color_.process(frameset);
  const rs2::frame depth_frame = aligned_frameset.get_depth_frame();
  const rs2::frame color_frame = aligned_frameset.get_color_frame();
  const cv::Size img_size(RGB_WIDTH, RGB_HEIGHT);
  const cv::Mat rgb_img(img_size, CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
  const cv::Mat depth_img(img_size, CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

  // copy realsense data pointer
  rgb_img.copyTo(ret.rgb_img);
  depth_img.convertTo(ret.depth_img, CV_32FC1, depth_scale_);
  ret.timestamp = MILLI2SEC(frameset.get_timestamp());

  return ret;
}

