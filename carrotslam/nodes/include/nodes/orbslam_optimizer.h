/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_optimizer.h
 * Date:   2015.10.04
 * Func:   ORB-SLAM tracking
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

#include <vector>
#include <set>
#include "core/carrot_slam.h"
#include "types/frame.h"
#include "types/feature.h"
#include "types/map.h"
#include "types/point.h"

#ifndef NODES_ORBSLAM_OPTIMIZER_H_
#define NODES_ORBSLAM_OPTIMIZER_H_

using namespace std;
using namespace Eigen;

namespace g2o {
class EdgeSE3ProjectXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ,
    VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError() {
    const VertexSE3Expmap* v1 =
        static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 =
        static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 =
        static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 =
        static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2) > 0.0;
  }

  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
};

}  // namespace g2o

namespace carrotslam {
namespace orbslam {
struct XYZ2UV {
  Eigen::Vector3f pos;
  Eigen::Vector2f px;
  float inv_sigma2;
  bool is_deleted;
  FeaturePtr current;
  FeaturePtr previous;
  XYZ2UV(const FeaturePtr& feat) {
    pos = feat->point->pos;
    px = feat->px;
    inv_sigma2 = Feature::inv_level_sigma2[feat->level];
    is_deleted = false;
    current = nullptr;
    previous = nullptr;
  }
};
typedef std::shared_ptr<XYZ2UV> XYZ2UVPtr;
//should change method with more specific name???
int poseOptimization(const CameraPtr& cam, std::vector<XYZ2UVPtr>& points, Sophus::SE3f& T_f_w);
}// namespace orbslam
}// namespace carrotslam

#endif /*  NODES_ORBSLAM_OPTIMIZER_H_ */
