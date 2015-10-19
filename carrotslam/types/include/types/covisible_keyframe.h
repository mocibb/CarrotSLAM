/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   covisible_keyframe.h
 * Date:   2015.10.11
 * Func:
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
#ifndef TYPES_COVISIBLE_KEYFRAME_H_
#define TYPES_COVISIBLE_KEYFRAME_H_
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "core/carrot_slam.h"
#include "types/pinhole_camera.h"
#include "types/frame.h"
#include <set>

/*
 * CovisibleKeyFrame用来直接描述Frame跟Frame之间的关系。
 *                     50(相互可见的点)
 *          frame1   ------ frame2
 *
 * 不用CovisibleKeyFrame时frame是这么相连的。
 *           frame1         frame2
 *              |              |
 *           feature1      feature2
 *               \            /
 *                 \         /
 *                   point
 *
 */
namespace carrotslam {
class PinholeCamera;
class Feature;
class CovisibleKeyFrame;

typedef std::shared_ptr<PinholeCamera> CameraPtr;
typedef std::shared_ptr<Feature> FeaturePtr;
typedef std::shared_ptr<CovisibleKeyFrame> CovisibleKeyFramePtr;

class CovisibleEdge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CovisibleKeyFramePtr vertex;
  float weight;
  friend bool operator>(const CovisibleEdge& lhs, const CovisibleEdge& rhs) {
    return lhs.weight > rhs.weight;
  }
};

class CovisibleKeyFrame : public Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void add(const CovisibleKeyFramePtr& frame, float weight);
  void remove(const CovisibleKeyFramePtr& frame);
  std::vector<CovisibleKeyFramePtr> getCovisibleKeyFrames();
  std::vector<CovisibleKeyFramePtr> getBestCovisibilityKeyFrames(const int &N);

 private:
  std::multiset<CovisibleEdge, std::greater<CovisibleEdge> > edges_;
};

}  // namespace carrotslam

#endif /* TYPES_COVISIBLE_KEYFRAME_H_ */
