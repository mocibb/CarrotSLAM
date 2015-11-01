/*!
 * Author: brad lucas liufuqiang_robot@hotmail.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   map_point.h
 * Date:   2015.11.1
 * Func:   map_point
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

#ifndef NODES_ORBSLAM_MAP_POINT_H
#define NODES_ORBSLAM_MAP_POINT_H

// std C++
#include <iostream>
#include <fstream>
#include <vector>
#include <set>

// boost
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// pcl
//

// carrotslam
#include "core/carrot_slam.h"
#include "types/dimage.h"
#include "types/frame.h"
#include "types/feature.h"
#include "types/point.h"

using namespace carrotslam;
namespace carrotslam {
/*! \class ORBSLAMMap
 * \brief mapping
 *
 */
class ORBSLAMMapPoint : public ISLAMNode
{
public:
	ORBSLAMMapPoint(const ISLAMEnginePtr& engine, const std::string& name);
	~ORBSLAMMapPoint();
	
	// 继承函数
	virtual RunResult run();
	
	bool check();
	
	bool isStart();
	
	bool isEnd();
	
	//自有函数
protected:
	void SetWorldPos(const cv::Mat &Pos);
	cv::Mat GetWorldPos();
	
	cv::Mat GetNormal();
	KeyFrame* GetReferenceKeyFrame();
	
	std::map<KeyFrame*, size_t> GetObservations();
	int Observations();
	
	void AddObservation(KeyFrame *pKF, size_t idx);
	void EraseObservation(KeyFrame *pKF);
	
	int GetIndexInKeyFrame(KeyFrame* pKF);
	bool IsInKeyFrame(KeyFrame *pKF);
	
	void SetBadFlag();
	bool IsBad();
	
	void Replace(MapPoint *pMP);
	
	void IncreaseVisible();
	void IncreaseFound();
	float GetFoundRatio();
	
	void ComputeDistinctiveDescriptors();
	
	cv::Mat GetDescriptor();
	
	void UpdateNormalAndDepth();
	
	float GetMinDistanceInvariance();
	float GetMaxDistanceInvariance();
	
	//数据成员
public:
	long unsigned int mnId_;
	static long unsigned int nNextId_;
	long int mnFirstKFid_;
	
	// Variables used by the tracking
	float mTrackProjX_;
	float mTrackProjY_;
	bool mbTrackInView_;
	int mnTrackScaleLevel_;
	float mTrackViewCos_;
	long unsigned int mnTrackReferenceForFrame_;
	long unsigned int mnLastFrameSeen_;
	
	// Variables used by local mapping
	long unsigned int mnBALocalForKF_;
	long unsigned int mnFuseCandidateForKF_;
	
	// Variables used by loop closing
	long unsigned int mnLoopPointForKF_;
	long unsigned int mnCorrectedByKF_;
	long unsigned int mnCorrectedReference_;
	
protected:
	// Position in absolute coordinates
	cv::Mat mWorldPos_;
	
	// KeyFrames observing the point and associated index in keyframe
	std::map<KeyFrame*, size_t> mObservations_;
	
	// Mean viewing direction
	cv::Mat mNormalVector_;
	
	// Best descriptor to fast matching
	cv::Mat mDescriptor_;
	
	// Reference KeyFrame
	KeyFrame *mpRefKF_;
	
	// Tracking counters
	int mnVisible_;
	int mnFound_;
	
	// Bad flag (we do not currently erase MapPoint from memory)
	bool mbBad_;
	
	// Scale invariance distances
	float mfMinDistance_;
	float mfMaxDistance_;
	
	Map *mpMap_;
	
	boost::mutex mMutexPos_;
	boost::mutex mMutexFeatures_;
	
};

} // end of namespace 

#endif // map_point.h