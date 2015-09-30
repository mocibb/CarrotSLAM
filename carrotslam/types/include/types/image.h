/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   image.h
 * Date:   2015.09.30
 * Func:   image object
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
#ifndef TYPES_IMAGE_H_
#define TYPES_IMAGE_H_

#include "core/carrot_slam.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace carrotslam {
/*! \brief image object that keep id
 *
 *
 *  Internally use cv::Mat of opencv for holding image object
 */
class Image : public ISLAMData {
 public:
  /*!
   the constructor of image
   @param img_path the path to image file
   */
  Image(const std::string& img_path)
      : id_(image_id_++) {
    image_ = cv::imread(img_path);
  }
  /*!
   the constructor of image
   @param image image object
   */
  Image(cv::Mat& image)
      : image_(image),
        id_(image_id_++) {
  }
  /*!
   return image object
   */
  cv::Mat& image() {
    return image_;
  }
  /*!
   return image object
   */
  const cv::Mat& image() {
    return image_;
  }

 protected:
  cv::Mat image_;          //!< image object
  static long image_id_;   //!< global counter for image class
};
}   // namespace carrotslam

#endif /* TYPES_IMAGE_H_ */
