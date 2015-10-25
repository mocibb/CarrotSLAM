/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include "types/pinhole_camera.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace carrotslam;

PinholeCamera::
PinholeCamera(double width, double height,
    double fx, double fy,
    double cx, double cy,
    double d0, double d1, double d2, double d3, double d4,
    double scale_factor) :
fx_(fx), fy_(fy), cx_(cx), cy_(cy),
distortion_(fabs(d0) > 0.0000001),
undist_map1_(height_, width_, CV_16SC2),
undist_map2_(height_, width_, CV_16SC2),
use_optimization_(false)
{
  width_ = width;
  height_= height;
  d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
  cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_,
      cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::~PinholeCamera() {
}

Eigen::Vector3d PinholeCamera::cam2world(const double& u,
                                         const double& v) const {
  Eigen::Vector3d xyz;
  if (!distortion_) {
    xyz[0] = (u - cx_) / fx_;
    xyz[1] = (v - cy_) / fy_;
    xyz[2] = 1.0;
  } else {
    cv::Point2f uv(u, v), px;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
    cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
    xyz[0] = px.x;
    xyz[1] = px.y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Eigen::Vector3d PinholeCamera::cam2world(const Eigen::Vector2d& uv) const {
  return cam2world(uv[0], uv[1]);
}

Eigen::Vector3d PinholeCamera::cam2world( const Eigen::Vector2d& uv, const short& depth ) const 
{
    double z = double( depth ) / scale_factor_;
    double x = ( uv[0] - cx_ ) * z / fx_; 
    double y = ( uv[1] - cy_ ) * z / fy_;
    return Eigen::Vector3d( x,y,z );
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector3d& xyz) const {
  Eigen::Vector2d xy;
  return world2cam(xy);
  //return world2cam(project2d(xyz));
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector2d& uv) const {
  Eigen::Vector2d px;
  if (!distortion_) {
    px[0] = fx_ * uv[0] + cx_;
    px[1] = fy_ * uv[1] + cy_;
  } else {
    double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
    x = uv[0];
    y = uv[1];
    r2 = x * x + y * y;
    r4 = r2 * r2;
    r6 = r4 * r2;
    a1 = 2 * x * y;
    a2 = r2 + 2 * x * x;
    a3 = r2 + 2 * y * y;
    cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;
    xd = x * cdist + d_[2] * a1 + d_[3] * a2;
    yd = y * cdist + d_[2] * a3 + d_[3] * a1;
    px[0] = xd * fx_ + cx_;
    px[1] = yd * fy_ + cy_;
  }
  return px;
}

void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified) {
  if (distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
  else
    rectified = raw.clone();
}

std::vector<cv::KeyPoint> PinholeCamera::UndistortKeyPoints(std::vector<cv::KeyPoint> vKeys)
{
  std::vector<cv::KeyPoint> vKeysUn;
    if(fabs(mDistCoef_.at<float>(0)) < 1e-3)
    {
        vKeysUn = vKeys;
        return vKeysUn;
    }

    // Fill matrix with points
    cv::Mat mat(vKeys.size(), 2, CV_32F);
    for(unsigned int i=0; i<vKeys.size(); i++)
    {
        mat.at<float>(i, 0) = vKeys[i].pt.x;
        mat.at<float>(i, 1) = vKeys[i].pt.y;
    }

    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, mK_, mDistCoef_, cv::Mat(), mK_);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    vKeysUn.resize(vKeys.size());
    for(unsigned int i=0; i<vKeys.size(); i++)
    {
        cv::KeyPoint kp = vKeys[i];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        vKeysUn[i] = kp;
    }

  return vKeysUn;
}

CvPoint PinholeCamera::UndistortSinglePoint(CvPoint cpSinglePoint)
{
  CvPoint cpSinglePointUn;
  if(fabs(mDistCoef_.at<float>(0)) < 1e-3)
  {
      cpSinglePointUn = cpSinglePoint;
      return cpSinglePointUn;
  }

  // Fill matrix with point
  cv::Mat mat(1, 2, CV_32F);
  mat.at<float>(0, 0) = cpSinglePoint.x;
  mat.at<float>(0, 1) = cpSinglePoint.y;

  // Undistort point
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, mK_, mDistCoef_, cv::Mat(), mK_);
  mat = mat.reshape(1);

  // Fill undistorted singlepoint
  cpSinglePointUn.x = mat.at<float>(0, 0);
  cpSinglePointUn.x = mat.at<float>(0, 1);

  return cpSinglePointUn;
}

void PinholeCamera::ComputeImageBounds(cv::Mat im)
{
    if(fabs(mDistCoef_.at<float>(0)) > 1e-3)
    {
        cv::Mat mat(4, 2, CV_32F);
        mat.at<float>(0, 0) = 0.0;     mat.at<float>(0, 1) = 0.0;
        mat.at<float>(1, 0) = im.cols; mat.at<float>(1, 1) = 0.0;
        mat.at<float>(2, 0) = 0.0;     mat.at<float>(2, 1) = im.rows;
        mat.at<float>(3, 0) = im.cols; mat.at<float>(3, 1) = im.rows;

        // Undistort corners
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK_, mDistCoef_, cv::Mat(), mK_);
        mat = mat.reshape(1);

        mnMinX_ = std::min(floor(mat.at<float>(0, 0)), floor(mat.at<float>(2, 0)));
        mnMaxX_ = std::max(ceil(mat.at<float>(1, 0)),  ceil(mat.at<float>(3, 0)));
        mnMinY_ = std::min(floor(mat.at<float>(0, 1)), floor(mat.at<float>(1, 1)));
        mnMaxY_ = std::max(ceil(mat.at<float>(2, 1)),  ceil(mat.at<float>(3, 1)));
    }
    else
    {
        mnMinX_ = 0;
        mnMaxX_ = im.cols;
        mnMinY_ = 0;
        mnMaxY_ = im.rows;
    }
}
