/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_tracking.h
 * Date:   2015.10.05
 * Func:   tracking module in ORB-SLAM
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
#ifndef NODES_ORBSLAM_TRACKING_H_
#define NODES_ORBSLAM_TRACKING_H_

#include <vector>
#include <set>
#include "core/carrot_slam.h"
#include "types/frame.h"
#include "types/feature.h"
#include "types/map.h"
#include "types/point.h"
#include "nodes/orbslam_optimizer.h"

namespace carrotslam {
namespace orbslam {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
/*!
 * \brief put feature in grid
 */
class FeatureGrid {

 public:
  FeatureGrid(float width_inv, float height_inv, int minX, int minY)
      : grid_element_width_inv_(width_inv),
        grid_element_height_inv_(height_inv),
        min_x_(minX),
        min_y_(minY) {
  }

  /*!
   *  set frame will cause regrid all feature of the frame
   *  !!!this method should call before use!!!
   */
  void setFrame(const FramePtr& frame);

  std::vector<int> getFeaturesInArea(float x, float y, float r, int minLevel,
                                     int maxLevel);

 private:
  bool posInGrid(const FeaturePtr &feat, int& posX, int& posY);

  std::vector<int> grid_[FRAME_GRID_COLS][FRAME_GRID_ROWS];
  FramePtr frame_;
  float grid_element_width_inv_;
  float grid_element_height_inv_;
  int min_x_;
  int min_y_;

};

/*! \brief camera motion tracking of ORBSLAM
 *  this module does almost same function in Tracking.cc of original ORBSLAM.
 *  with small modification.
 *  this module not include relocalization and extraction of keyframe.
 */
class ORBSLAMTracking : public ISLAMNode {
 public:
  /*!
   the constructor of ORBSLAMTracking
   @param engine
   */
  ORBSLAMTracking(const ISLAMEnginePtr& engine);

  ~ORBSLAMTracking() {
    if (feature_grid_ != nullptr) {
      delete feature_grid_;
    }
  }

  RunResult run();

  inline bool check();

  bool isStart();

  bool isEnd();

 protected:

  bool hasMotionModel();

  /*! estimate frame pose using previous velocity */
  bool trackWithMotionModel();
  /*! estimate frame pose by previous frame feature */
  bool trackPreviousFrame();

  bool trackLocalMap();

  /*! search for all the overlap frame */
  void updateReferenceKeyFrames(std::set<FramePtr>& overlap_frames);

  /*! search all the candidate feature in overlap frame */
  void updateReferencePoints(const std::set<FramePtr>& overlap_frame,
                             std::vector<XYZ2UVPtr>& points);

  /*! search matched feature of current frame in given window of estimated project location */
  int searchByProjection(const FramePtr& previous, const FramePtr& current,
                         std::vector<XYZ2UVPtr>& points, float th);

  /*! search matched feature of current frame with candidate point in previous keyframe */
  int searchByProjection(const FramePtr& current,
                         std::vector<XYZ2UVPtr>& points,
                         std::vector<XYZ2UVPtr>& out, const float th);

  /*! search matched feature of current frame in given window of feature of previous frame */
  int windowSearch(const FramePtr& previous, const FramePtr& current,
                   std::vector<XYZ2UVPtr>& points, int window_size,
                   int min_level = -1, int max_level = INT_MAX);

  /*! compute the best among features */
  inline void findBestMatch(const FramePtr& frame,
                            const std::vector<int>& feature_index,
                            cv::Mat& desc, int& best_idx, float& best_dist,
                            float& best2_dist);

  /*! compute the best among features */
  inline void findBestMatch(const FramePtr& frame,
                            const std::vector<int>& feature_index,
                            cv::Mat& desc, int& best_idx, float& best_dist,
                            float& best2_dist, int& best_level,
                            int& best2_level);

  /*! compute the index of top3 of rotation change of features */
  void computeThreeMaxima(std::vector<int>* histo, int &idx1, int &idx2,
                          int &idx3);

  FeatureGrid* feature_grid_;

  bool check_orientation_;
  bool motion_model_;
  float nn_ratio_;
  float dist_threshold_;
  int histogram_length_;

  Sophus::SE3f velocity_;

  FramePtr current_;
  FramePtr previous_;
  MapPtr map_;
  set<FramePtr> overlap_frames_;

 private:
  void checkOrientation(std::vector<int>* rot_hist,
                        const std::vector<XYZ2UVPtr>& points,
                        std::vector<XYZ2UVPtr>& out);

  std::vector<XYZ2UVPtr> points_;  //< temp use
};
}   // namespace orbslam
}   // namespace carrotslam

#endif /* NODES_ORBSLAM_TRACKING_H_ */
