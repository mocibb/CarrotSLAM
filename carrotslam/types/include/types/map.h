/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   map.h
 * Date:   2015.10.06
 * Func:   map object contains keyframes and mappoints
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
#ifndef TYPES_MAP_H_
#define TYPES_MAP_H_
#include <Eigen/Core>
#include "core/carrot_slam.h"
#include "types/frame.h"
#include "types/point.h"

namespace carrotslam {
class Feature;
class Map;

typedef std::shared_ptr<Feature> FeaturePtr;
typedef std::shared_ptr<Point> PointPtr;
typedef std::shared_ptr<Map> MapPtr;

class Map : public ISLAMData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector< FramePtr > key_frames;  //!<
  std::vector< PointPtr > points;      //!<
};

}  // namespace carrotslam

#endif /* TYPES_MAP_H_ */
