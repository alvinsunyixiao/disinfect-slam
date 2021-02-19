#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <popl.hpp>
#include <string>
#include <thread>

#include "cameras/l515.h"
#include "cameras/zed.h"
#include "modules/slam_module.h"
#include "modules/tsdf_module.h"
#include "utils/config_reader.hpp"
#include "utils/cuda/errors.cuh"
#include "utils/gl/renderer_base.h"
#include "utils/time.hpp"

class ImageRenderer : public RendererBase {
 public:
  ImageRenderer(const std::string& name, const ZED& zed,
                const std::shared_ptr<TSDFSystem>& tsdf,
                const std::string& config_file_path)
      : RendererBase(name),
        zed_(zed),
        tsdf_(tsdf),
        config_(YAML::LoadFile(config_file_path)),
        virtual_cam_(GetIntrinsicsFromFile(config_file_path), 360, 640) {
    ImGuiIO& io = ImGui::GetIO();
    io.FontGlobalScale = 2;
  }

  void DispatchInput() override {
    ImGuiIO& io = ImGui::GetIO();
    if (io.MouseWheel != 0) {
      follow_cam_ = false;
      const Eigen::Vector3f move_cam(0, 0, io.MouseWheel * .1);
      const Eigen::Quaternionf virtual_cam_R_world = virtual_cam_T_world_.GetR();
      const Eigen::Vector3f virtual_cam_t_world = virtual_cam_T_world_.GetT();
      virtual_cam_T_world_ = SE3<float>(virtual_cam_R_world, virtual_cam_t_world - move_cam);
    }
    if (!io.WantCaptureMouse && ImGui::IsMouseDragging(0) && tsdf_normal_.GetWidth()) {
      follow_cam_ = false;
      const ImVec2 delta = ImGui::GetMouseDragDelta(0);
      const Eigen::Vector2f delta_img(delta.x / io.DisplaySize.x * tsdf_normal_.GetWidth(),
                                      delta.y / io.DisplaySize.y * tsdf_normal_.GetHeight());
      const Eigen::Vector2f pos_new_img(io.MousePos.x / io.DisplaySize.x * tsdf_normal_.GetWidth(),
                                        io.MousePos.y / io.DisplaySize.y * tsdf_normal_.GetHeight());
      const Eigen::Vector2f pos_old_img = pos_new_img - delta_img;
      const Eigen::Vector3f pos_new_cam = virtual_cam_.intrinsics_inv * pos_new_img.homogeneous();
      const Eigen::Vector3f pos_old_cam = virtual_cam_.intrinsics_inv * pos_old_img.homogeneous();
      const Eigen::Vector3f pos_new_norm_cam = pos_new_cam.normalized();
      const Eigen::Vector3f pos_old_norm_cam = pos_old_cam.normalized();
      const Eigen::Vector3f rot_axis_cross_cam = pos_new_norm_cam.cross(pos_old_norm_cam);
      const float theta = acos(pos_new_norm_cam.dot(pos_old_norm_cam));
      const Eigen::Quaternionf R(Eigen::AngleAxisf(theta, rot_axis_cross_cam.normalized()));
      const SE3<float> pose_cam1_T_cam2(R, Eigen::Vector3f::Zero());
      virtual_cam_T_world_ = pose_cam1_T_cam2.Inverse() * virtual_cam_T_world_old_;
    } else if (!io.WantCaptureMouse && ImGui::IsMouseDragging(2)) {
      follow_cam_ = false;
      const ImVec2 delta = ImGui::GetMouseDragDelta(2);
      const Eigen::Vector3f translation(delta.x, delta.y, 0);
      const Eigen::Vector3f T = virtual_cam_T_world_old_.GetT();
      const Eigen::Quaternionf R = virtual_cam_T_world_old_.GetR();
      virtual_cam_T_world_ = SE3<float>(R, T + translation * .01);
    } else {
      virtual_cam_T_world_old_ = virtual_cam_T_world_;
    }
  }

  void Render() override {
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    // GUI
    ImGui::Begin("Menu");
    if (ImGui::Button("Follow Camera")) {
      follow_cam_ = true;
    }
    // compute
    const auto world_T_cam = zed_.GetWorld_T_Cam();
    cam_T_world_ = world_T_cam.Inverse();
    if (!tsdf_normal_.GetHeight() || !tsdf_normal_.GetWidth()) {
      tsdf_normal_.BindImage(virtual_cam_.img_h, virtual_cam_.img_w, nullptr);
    }
    if (follow_cam_) {
      static float step = 0;
      ImGui::SliderFloat("behind actual camera", &step, 0.0f, 3.0f);
      virtual_cam_T_world_ =
          SE3<float>(cam_T_world_.GetR(), cam_T_world_.GetT() + Eigen::Vector3f(0, 0, step));
    }
    // bounding cube query
    static unsigned int last_query_time = 0;
    static size_t last_query_amount = 0;
    static float x_range[2] = {-1., 1.};
    static float y_range[2] = {-1., 1.};
    static float z_range[2] = {-1., 1.};
    ImGui::DragFloat2("x range", x_range);
    ImGui::DragFloat2("y range", y_range);
    ImGui::DragFloat2("z range", z_range);
    if (ImGui::Button("Save TSDF")) {
      const BoundingCube<float> volumn = {x_range[0], x_range[1], y_range[0],
                                          y_range[1], z_range[0], z_range[1]};
      const auto st = GetTimestamp<std::chrono::milliseconds>();
      const auto voxel_pos_tsdf = tsdf_->Query(volumn);
      const auto end = GetTimestamp<std::chrono::milliseconds>();
      last_query_time = end - st;
      last_query_amount = voxel_pos_tsdf.size();
      std::ofstream fout("/tmp/data.bin", std::ios::out | std::ios::binary);
      fout.write((char*)voxel_pos_tsdf.data(), voxel_pos_tsdf.size() * sizeof(VoxelSpatialTSDF));
      fout.close();
    }
    ImGui::Text("Last queried %lu voxels, took %u ms", last_query_amount, last_query_time);
    // render
    const auto st = GetTimestamp<std::chrono::milliseconds>();
    //tsdf_->Render(virtual_cam_, virtual_cam_T_world_, &tsdf_normal_);
    CUDA_SAFE_CALL(cudaDeviceSynchronize());
    const auto end = GetTimestamp<std::chrono::milliseconds>();
    ImGui::Text("Rendering takes %lu ms", end - st);
    tsdf_normal_.Draw();
    ImGui::End();
  }

 private:
  const ZED& zed_;
  bool follow_cam_ = true;
  GLImage8UC4 tsdf_normal_;
  std::shared_ptr<SLAMSystem> slam_;
  std::shared_ptr<TSDFSystem> tsdf_;
  SE3<float> cam_T_world_ = SE3<float>::Identity();
  SE3<float> virtual_cam_T_world_ = SE3<float>::Identity();
  SE3<float> virtual_cam_T_world_old_ = SE3<float>::Identity();
  const YAML::Node config_;
  const CameraParams virtual_cam_;
};

void reconstruct(ZED* zed, const L515& l515,
                 const std::string& config_file_path) {
  // initialize TSDF
  auto TSDF = std::make_shared<TSDFSystem>(0.01, 0.06, 4, GetIntrinsicsFromFile(config_file_path),
                                           GetExtrinsicsFromFile(config_file_path));
  std::this_thread::sleep_for(std::chrono::seconds(1));

  ImageRenderer renderer("tsdf", *zed, TSDF, config_file_path);

  TimeSyncer<timed_pose_t, timed_rgbd_frame_t> syncer(
      [&] (const timed_pose_t& pose, const timed_rgbd_frame_t& rgbd) {
    if (pose.confidence > 0 && !rgbd.color_img.empty() && !rgbd.depth_img.empty()) {
      //std::cout << "Pose @ " << pose.timestamp << " ms" << std::endl;
      //std::cout << pose.world_T_cam.GetT() << std::endl;
      //std::cout << "RGBD @ " << rgbd.timestamp << " ms" << std::endl;
      //std::cout << "  RGB Size: " << rgbd.color_img.size << std::endl;
      //std::cout << "  Depth Size: " << rgbd.depth_img.size << std::endl << std::endl;

      cv::Mat color_img, depth_img;
      cv::resize(rgbd.color_img, color_img, cv::Size(), .5, .5);
      cv::resize(rgbd.depth_img, depth_img, cv::Size(), .5, .5);
      TSDF->Integrate(pose.world_T_cam.Inverse(), color_img, depth_img);
    }
  });

  std::mutex mtx_sync;

  std::thread t_pose([&]() {
    while (true) {
      const auto pose = zed->GetTimedPose();
      std::lock_guard<std::mutex> lock(mtx_sync);
      syncer.AddMeasurement1(pose);
    }
  });

  std::thread t_tsdf([&]() {
    while (true) {
      const auto rgbd_frame = l515.GetRGBDFrame();
      std::lock_guard<std::mutex> lock(mtx_sync);
      syncer.AddMeasurement2(rgbd_frame);
    }
  });

  renderer.Run();
  t_pose.join();
  t_tsdf.join();
}

int main(int argc, char* argv[]) {
  popl::OptionParser op("Allowed options");
  auto help = op.add<popl::Switch>("h", "help", "produce help message");
  auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
  auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");

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

  if (!config_file_path->is_set()) {
    std::cerr << "Invalid Arguments" << std::endl;
    std::cerr << std::endl;
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
  if (debug_mode->is_set())
    spdlog::set_level(spdlog::level::debug);
  else
    spdlog::set_level(spdlog::level::info);

  // initialize cameras
  ZED zed;
  L515 l515;
  // initialize slam
  reconstruct(&zed, l515, config_file_path->value());

  return EXIT_SUCCESS;
}
