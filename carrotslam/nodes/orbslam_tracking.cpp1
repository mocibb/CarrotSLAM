/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_tracking.cpp
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

#include "core/common.h"
#include "types/covisible_keyframe.h"
#include "nodes/orbslam_tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <glog/logging.h>

using namespace std;
using namespace Eigen;

namespace carrotslam {
namespace orbslam {
void FeatureGrid::setFrame(const FramePtr& frame) {
  //clear
  for (int x = 0; x < FRAME_GRID_COLS; x++) {
    for (int y = 0; y < FRAME_GRID_ROWS; y++) {
      grid_[x][y].clear();
    }
  }
  int posX, posY;
  for (int i = 0; i < frame->features.size(); i++) {
    if (posInGrid(frame->features[i], posX, posY))
      grid_[posX][posY].push_back(i);
  }
}

vector<int> FeatureGrid::getFeaturesInArea(float x, float y, float r,
                                           int min_level, int max_level) {
  vector<int> feat_idx;

  //TODO: x - minX_ - r may be need modify
  int min_cell_x = floor((x - min_x_ - r) * grid_element_width_inv_);
  int max_cell_x = ceil((x - min_x_ + r) * grid_element_width_inv_);
  int min_cell_y = floor((y - min_y_ - r) * grid_element_height_inv_);
  int max_cell_y = ceil((y - min_y_ + r) * grid_element_height_inv_);

  min_cell_x = max(0, min_cell_x);
  max_cell_x = min(FRAME_GRID_COLS - 1, max_cell_x);
  min_cell_y = max(0, min_cell_y);
  max_cell_y = min(FRAME_GRID_ROWS - 1, max_cell_y);
  if (min_cell_x >= FRAME_GRID_COLS || max_cell_x < 0
      || min_cell_y >= FRAME_GRID_ROWS || max_cell_y < 0)
    return feat_idx;

  bool bCheckLevels = true;
  if (min_level == -1 && max_level == -1)
    bCheckLevels = false;

  for (int ix = min_cell_x; ix <= max_cell_x; ix++) {
    for (int iy = min_cell_y; iy <= max_cell_y; iy++) {
      vector<int> cell = grid_[ix][iy];
      if (cell.empty())
        continue;

      for (size_t j = 0; j < cell.size(); j++) {
        FeaturePtr feat = frame_->features[cell[j]];
        if (bCheckLevels) {
          if (feat->level < min_level || feat->level > max_level)
            continue;
        }

        if (abs(feat->px.x() - x) > r || abs(feat->px.y() - y) > r)
          continue;

        feat_idx.push_back(cell[j]);
      }
    }
  }

  return feat_idx;
}

bool FeatureGrid::posInGrid(const FeaturePtr &feat, int& posX, int& posY) {
  posX = round((feat->px.x() - min_x_) * grid_element_width_inv_);
  posY = round((feat->px.y() - min_y_) * grid_element_height_inv_);

  //Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
    return false;

  return true;
}

ORBSLAMTracking::ORBSLAMTracking(const ISLAMEnginePtr& engine) {
  motion_model_ = getTypedValue<bool>(this, "motionModel");
  check_orientation_ = getTypedValue<bool>(this, "checkOrientation");
  nn_ratio_ = getTypedValue<float>(this, "nnRatio", 0.9);
  dist_threshold_ = getTypedValue<float>(this, "distThreshold", 100);
  histogram_length_ = getTypedValue<int>(this, "histogramLength", 30);
  feature_grid_ = nullptr;
}

/*
 * histo中保存特征点上一帧跟当前帧的角度差，
 * 这个角度差是由于相机移动造成的，所以都应该差不多。
 * 所以选择角度差一致最多的可以作为判断内点的依据。
 */
void ORBSLAMTracking::computeThreeMaxima(std::vector<int>* histo, int &idx1,
                                         int &idx2, int &idx3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < histogram_length_; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      idx3 = idx2;
      idx2 = idx1;
      idx1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      idx3 = idx2;
      idx2 = i;
    } else if (s > max3) {
      max3 = s;
      idx3 = i;
    }
  }

  if (max2 < 0.1f * (float) max1) {
    idx2 = -1;
  }
  if (max3 < 0.1f * (float) max1) {
    idx3 = -1;
  }
}

bool ORBSLAMTracking::hasMotionModel() {
  return false;
}

/*!
 * find best match feature in featureIndex
 */
inline void ORBSLAMTracking::findBestMatch(
    const FramePtr& frame, const std::vector<int>& feature_index, cv::Mat& desc,
    int& best_idx, float& best_dist, float& best2_dist, int& best_level,
    int& best2_level) {
  best_dist = INT_MAX;
  best2_dist = INT_MAX;
  best_dist = -1;
  //find best match
  for (auto cur_feat_idx : feature_index) {
    if (frame->features[cur_feat_idx].get() != nullptr)
      continue;

    cv::Mat d = frame->features[cur_feat_idx]->descriptor;
    int dist = Feature::descriptorDist(desc, d);
    if (dist < best_dist) {
      best2_dist = best_dist;
      best2_level = best_level;
      best_dist = dist;
      best_level = frame->features[cur_feat_idx]->level;
      best_idx = cur_feat_idx;
    } else if (dist < best2_dist) {
      best2_dist = dist;
      best2_level = frame->features[cur_feat_idx]->level;
    }
  }
}

inline void ORBSLAMTracking::findBestMatch(
    const FramePtr& frame, const std::vector<int>& feature_index, cv::Mat& desc,
    int& best_idx, float& best_dist, float& best2_dist) {
  int best_level, best2_level;
  findBestMatch(frame, feature_index, desc, best_idx, best_dist, best2_dist,
                best_level, best2_level);
}

inline void ORBSLAMTracking::checkOrientation(
    std::vector<int>* rot_hist, const std::vector<XYZ2UVPtr>& points,
    std::vector<XYZ2UVPtr>& out) {
  int idx1 = -1;
  int idx2 = -1;
  int idx3 = -1;
  computeThreeMaxima(rot_hist, idx1, idx2, idx3);
  out.clear();
  out.reserve(points.size());
  int idx[3] = { idx1, idx2, idx3 };
  for (int i = 0; i < 3; i++) {
    if (i != -1) {
      for (int j = 0; j < rot_hist[idx[i]].size(); j++) {
        out.push_back(points[rot_hist[idx[i]][j]]);
      }
    }
  }
}

/*!
 * compute correspondence using estimated transformation of current frame
 */
int ORBSLAMTracking::searchByProjection(const FramePtr& previous,
                                        const FramePtr& current,
                                        std::vector<XYZ2UVPtr>& out, float th) {
  std::vector<XYZ2UVPtr> points;
  // Rotation Histogram (to check rotation consistency)
  vector<int> rot_hist[histogram_length_];
  for (int i = 0; i < histogram_length_; i++)
    rot_hist[i].reserve(16);
  const float factor = 1.0f / histogram_length_;

  out.clear();
  out.reserve(previous->features.size());

  for (auto prev_feat : previous->features) {
    if (prev_feat->point->type == Point::TYPE_GOOD) {
      // Project
      Vector2f cur_px = current->cam->world2cam(
          current->T_f_w * prev_feat->point->pos);
      //check range
      if (!current->cam->isInFrame(cur_px))
        continue;

      int level = prev_feat->level;

      // Search in a window. Size depends on scale
      //float radius = th * CurrentFrame.mvScaleFactors[predictedLevel];
      float radius = th * Feature::scale_factors[level];

      //run in current frame
      vector<int> feat_idx = feature_grid_->getFeaturesInArea(
          static_cast<float>(cur_px[0]), static_cast<float>(cur_px[1]), radius,
          level - 1, level + 1);

      if (feat_idx.empty())
        continue;

      float best_dist = INT_MAX;
      float best2_dist = INT_MAX;
      int best_idx = -1;

      //find best match
      findBestMatch(current, feat_idx, prev_feat->descriptor, best_idx,
                    best_dist, best2_dist);

      if (best_dist <= dist_threshold_) {
        XYZ2UV* point = new XYZ2UV(prev_feat);
        point->current = current->features[best_idx];
        point->previous = prev_feat;
        points.push_back(XYZ2UVPtr(point));

        if (check_orientation_) {
          float rot = prev_feat->angle - current->features[best_idx]->angle;
          if (rot < 0.0)
            rot += 360.0f;
          int bin = static_cast<int>(round(rot * factor)) % histogram_length_;
          rot_hist[bin].push_back(points.size() - 1);  //current idx is points.size()-1
        }
      }
    }
  }

  //Apply rotation consistency
  if (check_orientation_) {
    checkOrientation(rot_hist, points, out);
  } else {
    out = points;
  }

  return out.size();
}

static inline float radiusByViewingCos(const float viewCos) {
  if (viewCos > 0.998)
    return 2.5;
  else
    return 4.0;
}

int ORBSLAMTracking::searchByProjection(const FramePtr& current,
                                        std::vector<XYZ2UVPtr>& points,
                                        std::vector<XYZ2UVPtr>& out,
                                        const float th) {
  out.clear();
  out.reserve(points.size());

  //TODO will need add some rotation consistency check?
  for (auto pnt : points) {
    // Project
    Vector2f cur_px = current->cam->world2cam(
        current->T_f_w * pnt->previous->point->pos);

    int level = pnt->previous->level;

    float viewCos;

    // The size of the window will depend on the viewing direction
    float radius = th * Feature::scale_factors[level] * radiusByViewingCos(viewCos);

    vector<int> feat_idx = feature_grid_->getFeaturesInArea(
        static_cast<float>(cur_px[0]), static_cast<float>(cur_px[1]), radius,
        level - 1, level);

    if (feat_idx.empty())
      continue;

    float best_dist = INT_MAX;
    float best2_dist = INT_MAX;
    int best_idx = -1;
    int best_level = -1;
    int best2_level = -1;

    findBestMatch(current, feat_idx, pnt->previous->descriptor, best_idx,
                  best_dist, best2_dist, best_level, best2_level);

    // Apply ratio to second match (only if best and second are in the same scale level)
    if (best_dist <= dist_threshold_) {
      if (best_level == best2_level && best_dist > nn_ratio_ * best2_dist)
        continue;

      XYZ2UV* point = new XYZ2UV(pnt->previous);
      point->current = current->features[best_idx];
      point->previous = pnt->previous;
      out.push_back(XYZ2UVPtr(point));
    }
  }

  return out.size();
}

int ORBSLAMTracking::windowSearch(const FramePtr& previous,
                                  const FramePtr& current,
                                  std::vector<XYZ2UVPtr>& out, int window_size,
                                  int min_level, int max_level) {
  std::vector<XYZ2UVPtr> points;
  // Rotation Histogram (to check rotation consistency)
  vector<int> rot_hist[histogram_length_];
  for (int i = 0; i < histogram_length_; i++)
    rot_hist[i].reserve(16);
  const float factor = 1.0f / histogram_length_;

  int nmatches = 0;
  points.clear();

  const bool bMinLevel = min_level > 0;
  const bool bMaxLevel = max_level < INT_MAX;

  for (int prev_idx = 0; prev_idx < previous->features.size(); prev_idx++) {
    FeaturePtr prev_feat = previous->features[prev_idx];

    if (prev_feat->point->type == Point::TYPE_GOOD)
      continue;

    int prev_level = prev_feat->level;

    if (bMinLevel)
      if (prev_level < min_level)
        continue;

    if (bMaxLevel)
      if (bMaxLevel > max_level)
        continue;

    vector<int> feat_idx = feature_grid_->getFeaturesInArea(
        static_cast<float>(prev_feat->px[0]),
        static_cast<float>(prev_feat->px[1]), window_size, prev_level,
        prev_level);

    if (feat_idx.empty())
      continue;

    cv::Mat d1 = prev_feat->descriptor;

    float best_dist = INT_MAX;
    float best2_dist = INT_MAX;
    int best_idx = -1;

    findBestMatch(current, feat_idx, d1, best_idx, best_dist, best2_dist);

    if (best_dist <= best2_dist * nn_ratio_ && best_dist <= dist_threshold_) {
      XYZ2UV* point = new XYZ2UV(prev_feat);
      point->current = current->features[best_idx];
      point->previous = prev_feat;
      points.push_back(XYZ2UVPtr(point));

      if (check_orientation_) {
        float rot = prev_feat->angle - current->features[best_idx]->angle;
        if (rot < 0.0)
          rot += 360.0f;
        int bin = static_cast<int>(round(rot * factor)) % histogram_length_;
        rot_hist[bin].push_back(points.size() - 1);  //current idx is points.size()-1
      }
    }
  }

  //Apply rotation consistency
  if (check_orientation_) {
    checkOrientation(rot_hist, points, out);
  } else {
    out = points;
  }

  return out.size();
}

bool ORBSLAMTracking::trackWithMotionModel() {
  //Compute current pose by motion model
  current_->T_f_w = velocity_ * previous_->T_f_w;

  // Project points seen in previous frame
  int matched = searchByProjection(previous_, current_, points_, 15);
  DLOG(INFO)<< "project search 1: matched=" << matched << endl;

  if (matched < 20)
    return false;

  // Optimize pose with all correspondences
  matched = poseOptimization(current_->cam, points_, current_->T_f_w);
  DLOG(INFO)<< "pose opt 1: matched=" << matched << endl;

  return matched >= 10;
}

bool ORBSLAMTracking::trackPreviousFrame() {
  // Search first points at coarse scale levels to get a rough initial estimate
  int min_level = 0;
  int max_level = Feature::scale_factors.size() - 1;
  if (map_->key_frames.size() > 5)
    min_level = max_level / 2 + 1;

  int matched = windowSearch(previous_, current_, points_, 200, min_level);
  DLOG(INFO)<< "window search 1: matched=" << matched << ", level=" << min_level << endl;

  // If not enough matches, search again without scale constraint
  if (matched < 10) {
    matched = windowSearch(previous_, current_, points_, 100, 0);
    DLOG(INFO)<< "window search 2: matched=" << matched << ", level=" << 0 << endl;
    if (matched < 10) {
      matched = 0;
    }
  }

  current_->T_f_w = previous_->T_f_w;
  // If enough correspondences, optimize pose and project points from previous frame to search more correspondences
  if (matched >= 10) {
    // Optimize trackPreviousFramepose with correspondences
    matched = poseOptimization(current_->cam, points_, current_->T_f_w);
    DLOG(INFO)<< "pose opt 1: matched=" << matched << endl;

    // Search by projection with the estimated pose
    matched = searchByProjection(previous_, current_, points_, 15);
    DLOG(INFO)<< "project search 1: matched=" << matched << endl;
  } else {
    //Last opportunity
    //这步可能没有意义,因为当前的Pose没变，所以检测出的特征点不会比windowSearch多。
    matched = searchByProjection(previous_, current_, points_, 50);
    DLOG(INFO) << "project search 2: matched=" << matched << endl;
  }

  if (matched < 10)
    return false;

  // Optimize pose again with all correspondences
  matched = poseOptimization(current_->cam, points_, current_->T_f_w);
  DLOG(INFO)<< "pose opt 2: matched=" << matched << endl;

  return matched >= 10;
}

void ORBSLAMTracking::updateReferenceKeyFrames(set<FramePtr>& overlap_frames) {
  // Each map point vote for the keyframes in which it has been observed
  map<FramePtr, int> keyframe_cnt;
  for (auto feat : current_->features) {
    if (feat->point) {
      for (auto obs : feat->point->observations) {
        keyframe_cnt[obs->frame]++;
      }
    }
  }

  int max = 0;
  FramePtr kf_max = NULL;

  //set<FramePtr> overlap_frames;

  // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
  for (auto frame : keyframe_cnt) {
    if (frame.second > max) {
      max = frame.second;
      kf_max = frame.first;
    }

    overlap_frames.insert(frame.first);
  }

  set<FramePtr> neigh_frames;
  // Include also some not-already-included keyframes that are neighbors to already-included keyframes
  for (auto frame : overlap_frames) {
    // Limit the number of keyframes
    if (overlap_frames.size() > 80)
      break;

    CovisibleKeyFrame* ckf = static_cast<CovisibleKeyFrame*>(frame.get());
    vector<CovisibleKeyFramePtr> neighs = ckf->getBestCovisibilityKeyFrames(10);

    for (auto neigh : neighs) {
      //TODO 为什么只增加一个，是否修改成增加全部会比较好。
      neigh_frames.insert(neigh);
      break;
    }

  }

  overlap_frames.insert(neigh_frames.begin(), neigh_frames.end());

  //overlap_frames_ = overlap_frames;

  //mpReferenceKF = pKFmax;
}

void ORBSLAMTracking::updateReferencePoints(const set<FramePtr>& overlap_frame,
                                            std::vector<XYZ2UVPtr>& points) {
  points.clear();
  std::set<int> point_set;

  for (auto frame : overlap_frame) {
    for (auto feat : frame->features) {
      if (feat->point->type == Point::TYPE_GOOD) {
        if (point_set.count(feat->point->id()) == 0) {
          point_set.insert(feat->point->id());
          XYZ2UV* point = new XYZ2UV(feat);
          point->current = nullptr;
          point->previous = feat;
          points.push_back(XYZ2UVPtr(point));
        }
      }
    }  // end feature
  }  // end frame
}

bool ORBSLAMTracking::trackLocalMap() {
  set<FramePtr> overlap_frames;
  std::vector<XYZ2UVPtr> points, matched_points;
  updateReferenceKeyFrames(overlap_frames);

  updateReferencePoints(overlap_frames, points);

  //TODO  不使用已经匹配的
  //TODO add this to searchByProjection
  //mCurrentFrame.isInFrustum(pMP,0.5)
  int th = 1;
  //
  // TODO 如果刚刚relocalise过，增大 th = 5
  //
  std::vector<XYZ2UVPtr> out;
  searchByProjection(current_, points, matched_points, th);

  int matched = poseOptimization(current_->cam, matched_points,
                                 current_->T_f_w);
  if (matched < 30)
    return false;
}

ISLAMNode::RunResult ORBSLAMTracking::run() {
  {
    //timer for tracking
    ScopeTime(LOG(INFO), "ORBSLAM tracking");
    engine_->getData<Frame>("currFrame", current_);
    engine_->getData<Frame>("prevFrame", previous_);
    engine_->getData<Map>("globalMap", map_);

    // System is initialized. Track Frame.
    bool bOK;

    if (!hasMotionModel()) {
      bOK = trackPreviousFrame();
    } else {
      bOK = trackWithMotionModel();
      if (!bOK)
        bOK = trackPreviousFrame();
    }

    // If we have an initial estimation of the camera pose and matching. Track the local map.
    if (bOK) {
      //copy points back to
      for (auto p : points_) {
        if (!p->is_deleted) {
          p->current->point = p->previous->point;
          p->current->point->observations.push_back(p->current);
        }
      }
    }
    // If tracking were good, check if we insert a keyframe
    if (bOK) {

      // We allow points with high innovation (considererd outliers by the Huber Function)
      // pass to the new keyframe, so that bundle adjustment will finally decide
      // if they are outliers or not. We don't want next frame to estimate its position
      // with those points so we discard them in the frame.
    }

    // Update motion model
    if (motion_model_) {
      if (bOK) {
        velocity_ = current_->T_f_w * previous_->T_f_w.inverse();
      } else
        velocity_ = Sophus::SE3f();
    }

    if (bOK) {
      return RUN_FINISH;
    } else {
      return RUN_FAILED;
    }
  }  // end timer

}

}  // namespace orbslam
}  // namespace carrotslam
