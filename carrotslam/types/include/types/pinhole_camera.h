/*
 * pinhole_camera.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#ifndef PINHOLE_CAMERA_H_
#define PINHOLE_CAMERA_H_

#include "camera_base.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <algorithm>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace carrotslam {
class PinholeCamera : public CameraBase {

 private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;   //!< is it pure pinhole model or has it radial distortion?
  double d_[5];  //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  //distCoeffs – Output vector of distortion coefficients  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.

  cv::Mat cvK_, cvD_;
  cv::Mat undist_map1_, undist_map2_;
  bool use_optimization_;
  Eigen::Matrix3d K_;
  Eigen::Matrix3d K_inv_;
  double scale_factor_;  //RGBD Scale Factor

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(double width, double height, double fx, double fy, double cx,
                double cy, double d0 = 0.0, double d1 = 0.0, double d2 = 0.0,
                double d3 = 0.0, double d4 = 0.0, double scale_factor = 1000.0);

  ~PinholeCamera();

  void
  initUnistortionMap();

  virtual Eigen::Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Eigen::Vector3d
  cam2world(const Eigen::Vector2d& px) const;

  virtual Eigen::Vector3d
  cam2world(const Eigen::Vector2d& uv, const short& depth) const;

  virtual Eigen::Vector2d
  world2cam(const Eigen::Vector3d& xyz_c) const;

  virtual Eigen::Vector2d
  world2cam(const Eigen::Vector2d& uv) const;

  virtual Eigen::Vector2f
  world2cam(const Eigen::Vector3f& xyz_c) const;

  virtual Eigen::Vector2f
  world2cam(const Eigen::Vector2f& uv) const;

  const Eigen::Vector2d focal_length() const {
    return Eigen::Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const {
    return fabs(4.0 * fx_ * fy_);
  }

  inline const Eigen::Matrix3d& K() const {
    return K_;
  }
  ;
  inline const Eigen::Matrix3d& K_inv() const {
    return K_inv_;
  }
  ;
  inline double fx() const {
    return fx_;
  }
  ;
  inline double fy() const {
    return fy_;
  }
  ;
  inline double cx() const {
    return cx_;
  }
  ;
  inline double cy() const {
    return cy_;
  }
  ;
  inline double d0() const {
    return d_[0];
  }
  ;
  inline double d1() const {
    return d_[1];
  }
  ;
  inline double d2() const {
    return d_[2];
  }
  ;
  inline double d3() const {
    return d_[3];
  }
  ;
  inline double d4() const {
    return d_[4];
  }
  ;

  void undistortImage(const cv::Mat& raw, cv::Mat& rectified);
  // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
  cv::Mat mK_; // Camera Inner Parameters
  cv::Mat mDistCoef_; // Distortion Parameters

  // Undistorted Image Bounds (computed once)
  int mnMinX_;
  int mnMaxX_;
  int mnMinY_;
  int mnMaxY_;

  std::vector<cv::KeyPoint> UndistortKeyPoints(std::vector<cv::KeyPoint> vKeys);
  CvPoint UndistortSinglePoint(CvPoint cpSinglePoint);
  void ComputeImageBounds(cv::Mat im);

  inline bool isInFrame(const Eigen::Vector2i & obs, int boundary = 0) const {
    if (obs[0] >= mnMinX_+boundary && obs[0] < mnMaxX_ - boundary && obs[1] >= mnMinY_+boundary
        && obs[1] < mnMaxY_ - boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Eigen::Vector2i &o, int boundary,
                        int level) const {
    Eigen::Vector2i obs = obs*(1 << level);
    if (obs[0] >= mnMinX_+boundary && obs[0] < mnMaxX_ - boundary && obs[1] >= mnMinY_+boundary
        && obs[1] < mnMaxY_ - boundary)
      return true;
    return false;
  }

  cv::Mat getCvMat() const {
      double camera_matrix[3][3] = 
      {
          {fx_, 0, cx_}, 
          {0, fy_, cy_},
          {0, 0, 1}
      };
      return cv::Mat(3,3, CV_64F, camera_matrix);
  }
};

} // namespace carrotslam
#endif /* PINHOLE_CAMERA_H_ */
