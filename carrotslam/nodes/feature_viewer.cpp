/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   feature_viewer.cpp
 * Date:   2015.10.19
 * Func:   show feature image window using opencv
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
#include "nodes/feature_viewer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <glog/logging.h>

namespace carrotslam {

using namespace ORB_SLAM;
using namespace cv;
using namespace std;

static Scalar string2color(const string& color_str) {
  if (color_str[0] != '#' && color_str.size() != 7) {
    return Scalar();
  }

  return Scalar(std::stoul("0x"+color_str.substr(1, 2), nullptr, 16),
                std::stoul("0x"+color_str.substr(3, 2), nullptr, 16),
                std::stoul("0x"+color_str.substr(5, 2), nullptr, 16));
}
/*!
 the constructor of FeatureViewer
 */
FeatureViewer::FeatureViewer(const ISLAMEnginePtr& engine,
                             const std::string& name)
    : ISLAMNode(engine, name) {
  feature_color_ = string2color(getTypedValue<string>(this, "featureColor", ""));
  point_color_ = string2color(getTypedValue<string>(this, "pointColor", ""));
  feature_draw_thickness_ = getTypedValue<int>(this, "featureDrawThickness", 1);
  point_draw_thickness_ = getTypedValue<int>(this, "pointDrawThickness", 1);
  point_font_face_ = getTypedValue<int>(this, "pointFontFace", 8);
  point_font_scale_ = getTypedValue<double>(this, "pointFontScale", cv::FONT_HERSHEY_PLAIN);
}

inline void FeatureViewer::drawFeature(Mat& img, const FeaturePtr& feat) {
  cv::Point center(feat->opx[0], feat->opx[1]);

  int radius = 1 << feat->level;

  circle(img, center, radius, feature_color_, feature_draw_thickness_, CV_AA);

  // default value of angle is -1
  if (feat->angle != -1) {
    float srcAngleRad = feat->angle * (float) CV_PI / 180.f;
    cv::Point orient(cvRound(cos(srcAngleRad) * radius),
                     cvRound(sin(srcAngleRad) * radius));
    line(img, center, center + orient, feature_color_, 1, CV_AA);
  }

  if (feat->point != nullptr) {
    cv::Point text_center;
    string text;
    int thickness = point_draw_thickness_;
    int baseline=0;
    Size size = getTextSize(text, point_font_face_,
                                point_font_scale_, thickness, &baseline);
    baseline += thickness;

    if (floor(text_center.x-size.width/2)<0) {
      text_center.x -= floor(text_center.x-size.width/2);
    }
    if (ceil(text_center.x + size.width/2) >= img.cols) {
      text_center.x += img.cols-1-ceil(text_center.x + size.width/2);
    }

    if (floor(text_center.y-size.height/2)<0) {
      text_center.y -= floor(text_center.y-size.height/2);
    }
    if (ceil(text_center.y + size.height/2) >= img.rows) {
      text_center.y += img.rows-1-ceil(text_center.y + size.height/2);
    }
    putText(img, text, text_center, point_font_face_, point_font_scale_,
            point_color_, point_draw_thickness_, 8);
  }

}

ISLAMNode::RunResult FeatureViewer::run() {
  LOG(INFO)<<"FeatureViewer run" << endl;
  FramePtr frame = getSLAMData<Frame>(engine_, "frame");

  if (frame.get() != nullptr) {
    Mat view_img = frame->img;

    for (auto feat : frame->features) {
      drawFeature(view_img, feat);
    }

    imshow("FeatureViewer", view_img);
    waitKey();
  }
}

} // namespace carrotslam

