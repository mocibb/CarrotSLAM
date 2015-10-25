/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   common.h
 * Date:   2015.09.30
 * Func:   common function used in CarrotSLAM
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
#ifndef CORE_COMMON_H_
#define CORE_COMMON_H_

#include <iostream>
#include <string>
#include <glog/logging.h>
#include <boost/date_time/posix_time/posix_time.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

// Sophus
#include <sophus/se3.hpp>
namespace carrotslam {

/** \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create an instance at the beginning of the function. Example:
 *
 * \code
 * {
 *   ScopeTime t1 (LOG(INFO), "calculation");
 *
 *   // ... perform calculation here
 * }
 * \endcode
 *
 * \ingroup common
 */
class ScopeTime {
 public:
  inline ScopeTime(std::ostream& out, const std::string& title)
      : out_(out),
        title_(title) {
    start_time_ = boost::posix_time::microsec_clock::local_time();
  }

  inline ScopeTime(std::ostream& out)
      : ScopeTime(out, "") {
  }
  inline ScopeTime(const std::string& title)
      : ScopeTime(std::cerr, title) {
  }
  inline ScopeTime()
      : ScopeTime(std::cerr, "") {
  }


  inline ~ScopeTime() {
    double val = this->getTime();
    out_ << title_ << " took " << val << "ms.\n";
  }

  /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
  inline double getTime() {
    boost::posix_time::ptime end_time =
        boost::posix_time::microsec_clock::local_time();
    return (static_cast<double>(((end_time - start_time_).total_milliseconds())));
  }

 protected:
  boost::posix_time::ptime start_time_;
  std::string title_;
  std::ostream & out_;
};

// some small tools function

/** \brief 从cv的旋转矩阵到Eigen::Isometry3d的转换
 */
inline Eigen::Isometry3d cvMat2Eigen( const cv::Mat& rvec, const cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen( R, r );

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    //Eigen::Translation<double,3> trans( tvec.at<double>(0,0), 
    //        tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle; 
    T(0,3)  = tvec.at<double>(0,0);
    T(1,3)  = tvec.at<double>(0,1);
    T(2,3)  = tvec.at<double>(0,2);

    return T;
}

/** \brief convert from cv::Mat tvec and rvec to Sophus::SE3f 
 */
inline Sophus::SE3f     cvMat2SophusSE3f( const cv::Mat& rvec, const cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3f r;
    cv::cv2eigen( R, r );

    Sophus::SE3f T( r, Eigen::Vector3f (
                tvec.at<double> (0,0), 
                tvec.at<double> (0,1), 
                tvec.at<double> (0,2)  ) );

    return T;
    
}

}//end of namespace

#endif /* CORE_COMMON_H_ */
