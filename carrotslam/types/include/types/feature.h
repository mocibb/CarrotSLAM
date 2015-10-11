/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   feature.h
 * Date:   2015.10.02
 * Func:   image feature that may cross multi-frame
 *
 *
 * The MIT License (MIT)
 * Copyright (c) 2015 CarrotSLAM Group
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef TYPES_FEATURE_H_
#define TYPES_FEATURE_H_
#include <Eigen/Core>
#include <memory>
#include <vector>
#include "core/carrot_slam.h"
#include <opencv2/core/core.hpp>
#include "types/point.h"

namespace carrotslam {

class Point;
class Frame;
class Feature;

typedef std::shared_ptr<Point> PointPtr;
typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<Feature> FeaturePtr;

//class Camera : public ISLAMData {
// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  double fx;
//  double fy;
//  double cx;
//  double cy;
//};

/*! \brief
 *
 *  Feature是依赖于Frame
 *  Feature class in svo, MapPoint in orbslam
 *
 */
class Feature : public ISLAMData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2f px;              //!<     畸变矫正后的坐标 Undisorted coordinates in pixels on pyramid level 0.
  int half_size;                   //!<     orbslam时half_size=0， svo时half_size=4
  Eigen::Vector3f f;               //!<     Unit-bearing vector of the feature.
  cv::Mat descriptor;              //!<     ORBSLAM
  float angle;                     //!<     ORBSLAM
  float min_depth;                 //!<     MapPoint.mfMinDistance;
  float max_depth;                 //!<     MapPoint.mfMaxDistance;
  int level;                       //!<     KeyPoint.octave
  FramePtr frame;
  PointPtr point;
  static std::vector<float> scale_factors;
  static std::vector<float> inv_level_sigma2;
  static float (*descriptorDist)(cv::Mat d1, cv::Mat d2);
};


}  // namespace carrotslam

#endif /* TYPES_FEATURE_H_ */
