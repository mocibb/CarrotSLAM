/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   covisible_keyframe.cpp
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
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include "core/carrot_slam.h"
#include "types/pinhole_camera.h"
#include "types/covisible_keyframe.h"
#include <set>


namespace carrotslam {

void CovisibleKeyFrame::add(const CovisibleKeyFramePtr& frame, float weight) {
  CovisibleEdge edge;
  edge.vertex = frame;
  edge.weight = weight;
  edges_.insert(edge);
}

void CovisibleKeyFrame::remove(const CovisibleKeyFramePtr& frame) {
  for (auto e : edges_) {
    if (e.vertex->id_ == frame->id_) {
      edges_.erase(e);
      break;
    }
  }
}


std::vector<CovisibleKeyFramePtr> CovisibleKeyFrame::getCovisibleKeyFrames() {
  std::vector<CovisibleKeyFramePtr> frames;

  for (auto e : edges_) {
    frames.push_back(e.vertex);
  }

  return frames;
}

std::vector<CovisibleKeyFramePtr> CovisibleKeyFrame::getBestCovisibilityKeyFrames(const int &N){
  std::vector<CovisibleKeyFramePtr> frames;

  for (auto e : edges_) {
    frames.push_back(e.vertex);
    if (frames.size() >= N)
      break;
  }

  return frames;
}
}  // namespace carrotslam

