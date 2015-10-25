/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   frame.h
 * Date:   2015.10.02
 * Func:   frame is composed of  camera image and camera pose
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
#ifndef TYPES_FRAME_H_
#define TYPES_FRAME_H_

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include "core/carrot_slam.h"
#include "types/pinhole_camera.h"
#include "types/feature.h"

namespace carrotslam {
class PinholeCamera;
class Feature;

typedef std::shared_ptr<PinholeCamera> CameraPtr;
typedef std::shared_ptr<Feature> FeaturePtr;

class Frame : public ISLAMData {
 public:
     Frame() : ISLAMData( 0 ) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<cv::Mat> img_pyramids; //!< 灰度图像金字塔 0是最精细的级别(无缩放的级别)
  cv::Mat img;
  Sophus::SE3f T_f_w; //!< 世界坐标到Frame坐标系变换  Transform (f)rame from (w)orld.
  bool is_keyframe = false;   //!<
  std::vector< FeaturePtr > features; //!< 图像特征 image feature
  CameraPtr cam; //!< 相机模型 Camera model.
};

}  // namespace carrotslam

#endif /* TYPES_FRAME_H_ */
