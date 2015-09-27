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

#include <opencv2/opencv.hpp>

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

//Camera Struct
struct CAMERA {
  double dFx; //Focal Length x
  double dFy; //Focal Length y
  double dCx; //Lense Offset x
  double dCy; //Lense Offset y
  double dScaleFactor; //RGBD Scale Factor
  
  int nSensorResolutionX; //CCD(CMOS) Resolution x
  int nSensorResolutionY; //CCD(CMOS) Resolution y
  double dPixelElemSizeX; //Pixel Element Size(mm, x)
  double dPixelElemSizeY; //Pixel Element Size(mm, y)
  
  cv::Mat mDistortionMat; //Camera Distortion Rectify Matrix
  
  double dMaxFrameRate; //Max Frame Rate
  double dCommonFrameRate; //Common Frame Rate
  
  E_CAMERA_INTERFACE eCameraInterface; //Camera Interface
  E_VIDEO_FORMAT eVideoFormat; //Video Format
  E_LENSE_TYPE eLenseType; //Lense Type
  double dLenseAngleX; //Lense Angle(x，Degree)
  E_CAMERA_TYPE eCameraType; //Camera Type
};
