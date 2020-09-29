#include <spdlog/spdlog.h>

#include "modules/tsdf_module.h"

TSDFSystem::TSDFSystem(float voxel_size, float truncation, float max_depth,
                       const CameraIntrinsics<float> &intrinsics)
  : tsdf_(voxel_size, truncation), max_depth_(max_depth), intrinsics_(intrinsics),
    t_(&TSDFSystem::Run, this) {}

TSDFSystem::~TSDFSystem() {
  {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_ = true;
  }
  t_.join();
}

void TSDFSystem::Integrate(const SE3<float> &cam_P_world,
                           const cv::Mat &img_rgb, const cv::Mat &img_depth,
                           const cv::Mat &img_ht, const cv::Mat &img_lt) {
  std::lock_guard<std::mutex> lock(mtx_queue_);
  inputs_.push(std::make_unique<TSDFSystemInput>(cam_P_world, img_rgb, img_depth, img_ht, img_lt));
}

void TSDFSystem::Render(const CameraParams &virtual_cam,
                        const SE3<float> cam_P_world,
                        GLImage8UC4 *img_normal) {
  std::lock_guard<std::mutex> lock(mtx_read_);
  tsdf_.RayCast(max_depth_, virtual_cam, cam_P_world, nullptr, img_normal);
}

void TSDFSystem::Run() {
  while (true) {
    // check for termination
    {
      std::lock_guard<std::mutex> lock(mtx_terminate_);
      if (terminate_)
        return;
    }
    // pop from input queue
    std::unique_ptr<TSDFSystemInput> input;
    {
      std::lock_guard<std::mutex> lock(mtx_queue_);
      if (inputs_.size() > 10)
        spdlog::warn("[TSDF System] Processing cannot catch up (input size: {})", inputs_.size());
      if (inputs_.empty())
        continue;
      input = std::move(inputs_.front());
      inputs_.pop();
    }
    // tsdf integration
    {
      std::lock_guard<std::mutex> lock(mtx_read_);
      tsdf_.Integrate(input->img_rgb, input->img_depth, input->img_ht, input->img_lt,
                      max_depth_, intrinsics_, input->cam_P_world);
    }
  }
}
