/*!
 * Author: lfq liufuqiang_robot@hotmail.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   Camera.h
 * Date:   2015.09.27
 * Func:   Camera type definition
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

#ifndef CAMERA_BASE_H_
#define CAMERA_BASE_H_

#include <Eigen/Core>

namespace carrotslam {
//Camera Interface
enum E_CAMERA_INTERFACE {
  E_INTERFACE_USB2 = 0,
  E_INTERFACE_USB3,
  E_INTERFACE_GIGE
};

//Camera Type
enum E_CAMERA_TYPE {
  E_CAMERA_TYPE_RGB = 0,
  E_CAMERA_TYPE_RGBD
};

//Video Format
enum E_VIDEO_FORMAT {
  E_VIDEO_FORMAT_PAL = 0,
  E_VIDEO_FORMAT_NTSC,
};

//Lense Type
enum E_LENSE_TYPE {
  E_LENSE_TYPE_GENERAL = 0,
  E_LENSE_TYPE_FISHEYE
};

class CameraBase {
 public:

  int width_;   // TODO cannot be const because of omni-camera model
  int height_;

  double pixel_elem_size_x_; //Pixel Element Size(mm, x)
  double pixel_elem_size_y_; //Pixel Element Size(mm, y)

  double max_frame_rate_; //Max Frame Rate
  double common_frame_rate_; //Common Frame Rate

  E_CAMERA_INTERFACE camera_interface_;  //Camera Interface
  E_VIDEO_FORMAT video_format_;  //Video Format
  E_LENSE_TYPE lense_type_;  //Lense Type
  double lense_angle_x_;  //Lense Angle(x，Degree)
  E_CAMERA_TYPE camera_type_;  //Camera Type

 public:

  CameraBase() {
  }
  ;  // need this constructor for omni camera
  CameraBase(int width, int height)
      : width_(width),
        height_(height) {
  }
  ;
  CameraBase(int width, int height, double pixel_elem_size_x,
             double pixel_elem_size_y)
      : width_(width),
        height_(height),
        pixel_elem_size_x_(pixel_elem_size_x),
        pixel_elem_size_y_(pixel_elem_size_y) {
  }
  ;
  CameraBase(int width, int height, double pixel_elem_size_x,
             double pixel_elem_size_y, double max_frame_rate,
             double common_frame_rate, E_CAMERA_INTERFACE camera_interface,
             E_VIDEO_FORMAT video_format, E_LENSE_TYPE lense_type,
             double lense_angle_x, E_CAMERA_TYPE camera_type)
      : width_(width),
        height_(height),
        pixel_elem_size_x_(pixel_elem_size_x),
        pixel_elem_size_y_(pixel_elem_size_y),
        max_frame_rate_(max_frame_rate),
        common_frame_rate_(common_frame_rate),
        camera_interface_(camera_interface),
        video_format_(video_format),
        lense_type_(lense_type),
        lense_angle_x_(lense_angle_x),
        camera_type_(camera_type){
  }
  ;

  virtual ~CameraBase() {
  }
  ;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d
  cam2world(const double& x, const double& y) const = 0;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Eigen::Vector3d
  cam2world(const Eigen::Vector2d& px) const = 0;

  virtual Eigen::Vector2d
  world2cam(const Eigen::Vector3d& xyz_c) const = 0;

  /// projects unit plane coordinates to camera coordinates
  virtual Eigen::Vector2d
  world2cam(const Eigen::Vector2d& uv) const = 0;

  virtual double
  errorMultiplier2() const = 0;

  virtual double
  errorMultiplier() const = 0;

  inline int width() const {
    return width_;
  }

  inline int height() const {
    return height_;
  }

  inline bool isInFrame(const Eigen::Vector2i & obs, int boundary = 0) const {
    if (obs[0] >= boundary && obs[0] < width() - boundary && obs[1] >= boundary
        && obs[1] < height() - boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Eigen::Vector2i &obs, int boundary,
                        int level) const {
    if (obs[0] >= boundary && obs[0] < width() / (1 << level) - boundary
        && obs[1] >= boundary && obs[1] < height() / (1 << level) - boundary)
      return true;
    return false;
  }
};
} // namespace carrotslam
#endif /* CAMERA_BASE_H_ */
