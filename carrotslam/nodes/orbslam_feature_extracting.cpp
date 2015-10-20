/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_feature_extracting.cpp
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

#include "core/common.h"
#include "types/image.h"
#include "types/dimage.h"
#include "nodes/orbslam_feature_extracting.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <glog/logging.h>

namespace carrotslam {

using namespace ORB_SLAM;
using namespace cv;
using namespace std;

/*!
 the constructor of ORBSLAMFeatureExtracting
 @param using ORBextractor to get ORB features
 */
ORBSLAMFeatureExtracting::ORBSLAMFeatureExtracting(const ISLAMEnginePtr& engine, const std::string& name)
    : ISLAMNode(engine, name) {
  feature_number_ = getTypedValue<int>(this, "featureNumber", 1000);
  scale_factor_ = getTypedValue<float>(this, "scaleFactor", 1.2);
  pyramid_level_ = getTypedValue<int>(this, "pyramidLevel", 8);
  fast_feature_threshold_ = getTypedValue<float>(this, "fastFeatureThreshold", 20);
  score_type_ = getTypedValue<int>(this, "scoreType", 1);

  extractor_ = new ORBextractor(feature_number_, scale_factor_, pyramid_level_,
                                score_type_, fast_feature_threshold_);

  for (int i = 0; i < pyramid_level_; i++) {
    Feature::scale_factors.push_back(powf(scale_factor_, i));
  }
  //Feature::inv_level_sigma2;
  //Feature::descriptorDist;

}

ISLAMNode::RunResult ORBSLAMFeatureExtracting::run() {
  LOG(INFO) << "ORBSLAM Feature Extracting" << endl;
  ISLAMDataPtr data;
  engine_->getData("image", data);
  Mat img;
  if (data.get() != nullptr) {
    img = static_cast<Image*>(data.get())->image();
  } else {
    engine_->getData("dimage", data);
    if (data.get() != nullptr) {
      img = static_cast<DImage*>(data.get())->color_image();
    } else {
      throw runtime_error("not image found!");
    }
  }

  vector<KeyPoint> keypoints;
  Mat descriptors;
  {
    ScopeTime(LOG(INFO), "extracting feature");
    (*extractor_)(img, Mat(), keypoints, descriptors);
  }

  FramePtr frame(new Frame);

  for (int i = 0; i < keypoints.size(); i++) {
    FeaturePtr feat(new Feature);
    feat->level = keypoints[i].octave;
    feat->angle = keypoints[i].angle;
    feat->frame = frame;
    feat->descriptor = descriptors.row(i);
    feat->px << keypoints[i].pt.x, keypoints[i].pt.y;
    feat->half_size = 0;
    frame->features.push_back(feat);
  }

  setSLAMData(engine_, "frame", frame);

}

}   // namespace carrotslam

