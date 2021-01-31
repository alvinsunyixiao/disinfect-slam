#pragma once

#include <cuda_runtime.h>

#include <Eigen/Dense>

/**
 * @brief template class for storing linear camera calibration
 *
 * @tparam T  paramter data type
 */
template <typename T>
class CameraIntrinsics {
 public:
  /**
   * @brief construct linear calibration from parameter
   *
   * @param fx  x dimension focal length, in [pixel]
   * @param fy  y dimension focal length, in [pixel]
   * @param cx  x dimension principle point, in [pixel]
   * @param cy  y dimension principle point, in [pixel]
   */
  __device__ __host__ CameraIntrinsics(const T& fx, const T& fy, const T& cx, const T& cy)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

  /**
   * @brief optimized calibration matrix inverse
   *
   * @return inversed linear calibration matrix
   */
  __device__ __host__ CameraIntrinsics<T> Inverse() const {
    const T fx_inv = 1 / fx_;
    const T fy_inv = 1 / fy_;
    const T cx = cx_;
    const T cy = cy_;
    return CameraIntrinsics<T>(fx_inv, fy_inv, -cx * fx_inv, -cy * fy_inv);
  }

  /**
   * @brief project point on to pixel plane
   *
   * @param vec3  3D point to be projected
   *
   * @return homogeneous image plane coordinate
   */
  __device__ __host__ Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1>& vec3) const {
    Eigen::Matrix<T, 3, 1> ret;
    ret << fx_ * vec3[0] + cx_ * vec3[2], fy_ * vec3[1] + cy_ * vec3[2], vec3[2];
    return ret;
  }

 private:
  const T fx_;
  const T fy_;
  const T cx_;
  const T cy_;
};

class CameraParams {
 public:
  CameraIntrinsics<float> intrinsics;
  CameraIntrinsics<float> intrinsics_inv;
  int img_h;
  int img_w;

 public:
  __device__ __host__ CameraParams(const CameraIntrinsics<float>& intrinsics_, int img_h_,
                                   int img_w_)
      : img_h(img_h_),
        img_w(img_w_),
        intrinsics(intrinsics_),
        intrinsics_inv(intrinsics_.Inverse()) {}
};
