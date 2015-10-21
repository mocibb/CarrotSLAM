/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_feature_extracting.h
 * Date:   2015.10.17
 * Func:   extract feature in image and put to frame data to context
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
#ifndef NODES_ORBSLAM_FEATURE_EXTRACTING_H_
#define NODES_ORBSLAM_FEATURE_EXTRACTING_H_

#include <vector>
#include <set>
#include "core/carrot_slam.h"
#include "types/frame.h"
#include "types/feature.h"
#include "types/map.h"
#include "types/point.h"
#include "nodes/ORBextractor.h"

namespace carrotslam {
/*! \brief using ORBextractor to get ORB features
 *
 */
class ORBSLAMFeatureExtracting : public ISLAMNode {
 public:
  ORBSLAMFeatureExtracting(const ISLAMEnginePtr& engine, const std::string& name);

  ~ORBSLAMFeatureExtracting(){
	  if (extractor_ != nullptr)
		  delete extractor_;
  };

  RunResult run();

  inline bool check() {
    return true;
  }

  bool isStart() {
    return true;
  }

  bool isEnd() {
    return true;
  }

 private:

  ORB_SLAM::ORBextractor* extractor_;

  int feature_number_;
  float scale_factor_;
  int pyramid_level_;
  float fast_feature_threshold_;
  int score_type_;

};
}   // namespace carrotslam

#endif /* NODES_ORBSLAM_FEATURE_EXTRACTING_H_ */
