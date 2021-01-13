#pragma once

#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>

#include "utils/cuda/lie_group.cuh"
#include "utils/cuda/camera.cuh"
#include "utils/gl/image.h"
#include "utils/tsdf/voxel_hash.cuh"

template <typename T>
struct BoundingCube {
  Vector3<T> lower;
  Vector3<T> upper;

  __host__ __device__ BoundingCube(const Vector3<T> &lower, const Vector3<T> &upper)
    : lower(lower), upper(upper) {}

  template <typename Tout = T>
  __host__ __device__ BoundingCube<Tout> Scale(T scale) const {
    return BoundingCube<Tout>((lower * scale).template cast<Tout>(),
                              (upper * scale).template cast<Tout>());
  }
};


class TSDFGrid {
 public:
  TSDFGrid(float voxel_size, float truncation);
  ~TSDFGrid();

  void Integrate(const cv::Mat &img_rgb, const cv::Mat &img_depth,
                 const cv::Mat &img_ht, const cv::Mat &img_lt,
                 float max_depth,
                 const CameraIntrinsics<float> &intrinsics,
                 const SE3<float> &cam_P_world);

  void RayCast(float max_depth,
               const CameraParams &virtual_cam,
               const SE3<float> &cam_P_world,
               GLImage8UC4 *tsdf_rgba = NULL, GLImage8UC4 *tsdf_normal = NULL);

  std::vector<Vector3<float>> GetMesh(const BoundingCube<float> &volumn, float cube_len);

  std::vector<VoxelSpatialTSDF> GatherValid();

  std::vector<VoxelSpatialTSDF> GatherVoxelsSparse(const BoundingCube<float> &volumn);

  std::vector<float> GatherVoxelsContiguous(const BoundingCube<float> &volumn,
                                            BoundingCube<short> *voxel_bound);

 protected:
  void Allocate(const cv::Mat &img_rgb, const cv::Mat &img_depth, float max_depth,
                const CameraParams &cam_params, const SE3<float> &cam_P_world);

  int GatherVisible(float max_depth,
                    const CameraParams &cam_params, const SE3<float> &cam_P_world);

  int GatherBlock();

  void UpdateTSDF(int num_visible_blocks, float max_depth,
                  const CameraParams &cam_params, const SE3<float> &cam_P_world);

  void SpaceCarving(int num_visible_blocks);

  cudaStream_t stream_;
  cudaStream_t stream2_;
  // voxel grid params
  VoxelHashTable hash_table_;
  const float voxel_size_;
  const float truncation_;

  // visibility buffer
  VoxelBlock *visible_blocks_;
  int *visible_mask_;
  int *visible_indics_;
  int *visible_indics_aux_;
  // image data buffer
  uchar3 *img_rgb_;
  float *img_depth_;
  float *img_ht_;
  float *img_lt_;
  float *img_depth_to_range_;
  // renderer buffer
  uchar4 *img_tsdf_rgba_;
  uchar4 *img_tsdf_normal_;
};

