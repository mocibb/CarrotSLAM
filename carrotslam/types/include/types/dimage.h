/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   dimage.h
 * Date:   2015.09.30
 * Func:   rgbd image object
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
#ifndef TYPES_DIMAGE_H_
#define TYPES_DIMAGE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "core/carrot_slam.h"
#include <glog/logging.h>

namespace carrotslam {
/*! \brief rgbd image object that keep id
 *
 *
 *  Internally use cv::Mat of opencv for holding color and depth image
 */
class DImage : public ISLAMData {
 public:
  /*!
   the constructor of DImage
   @param color_image_path the path to color image file
   @param depth_image_path the path to depth image file
   */
  DImage(const std::string& color_image_path,
         const std::string& depth_image_path)
      : ISLAMData(image_id_++) {
    color_image_ = cv::imread(color_image_path);
    depth_image_ = cv::imread(depth_image_path);
  }
  /*!
   the constructor of DImage
   @param color_image color image object
   @param depth_image depth image object
   */
  DImage(cv::Mat& color_image, cv::Mat& depth_image)
      : ISLAMData(image_id_++),
        color_image_(color_image),
        depth_image_(depth_image) {
  }
  DImage(DImage& other)
      : ISLAMData(image_id_++),
        color_image_(other.color_image()),
        depth_image_(other.depth_image()) {
    LOG(INFO)<<"copy constructor of DImage is called." << std::endl;
  }
  ~DImage() {
    LOG(INFO)<<"deconstructor of DImage is called." << std::endl;
  }
  /*!
   return color image object
   */
  cv::Mat& color_image() {
    return color_image_;
  }
  /*!
   return color image object
   */
  const cv::Mat& color_image() const {
    return color_image_;
  }

  /*!
   return depth image object
   */
  cv::Mat& depth_image() {
    return depth_image_;
  }
  /*!
   return depth image object
   */
  const cv::Mat& depth_image() const {
    return depth_image_;
  }

 protected:
  cv::Mat color_image_;    //!< color image object
  cv::Mat depth_image_;    //!< depth image object
  static long image_id_;   //!< global counter for DImage class
};
}   // namespace carrotslam

#endif /* TYPES_DIMAGE_H_ */
