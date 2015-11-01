/*!
 * Author: brad lucas liufuqiang_robot@hotmail.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   map.h
 * Date:   2015.11.1
 * Func:   map
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
 
#ifndef NODES_ORBSLAM_MAP_H
#define NODES_ORBSLAM_MAP_H

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
class ORBSLAMMap : public ISLAMNode
{
public:
	ORBSLAMMap(const ISLAMEnginePtr& engine, const std::string& name);
	~ORBSLAMMap();
	
	// 继承函数
	virtual RunResult run();
	
	bool check();
	
	bool isStart();
	
	bool isEnd();
	
	//自有函数
protected:
	//Set 方法
	void AddMapPoint(MapPoint *pMP);
	void AddKeyFrame(KeyFrame *pKF);
	void EraseMapPoint(MapPoint *pMP);
	void EraseKeyFrame(KeyFrame *pKF);
	void SetCurrentCameraPose(cv::Mat Tcw);
	void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
	void SetReferenceKeyFrames(const std::vector<KeyFrame*> &vpKFs);
	
	//Get 方法
	std::vector<MapPoint*> GetAllMapPoints();
	std::vector<KeyFrame*> GetAllKeyFrames();
	cv::Mat GetCameraPose();
	std::vector<MapPoint*> GetReferenceMapPoints();
	std::vector<KeyFrame*> GetReferenceKeyFrames();
	
	//得到点数
	int MapPointsInMap();
	int KeyFramesInMap();
	
	void SetFlagAfterBA();
	bool IsMapUpdated();
	void ResetUpdated();
	
	unsigned int GetMaxKFid();
	
	void clear();
	
	//数据成员
protected:
	std::set<MapPoint*> mspMapPoints_;
	std::set<KeyFrame*> mspKeyFrames_;
	
	std::vector<MapPoint*> mvpReferenceMapPoints_;
	
	unsigned int mnMaxKFid_;
	
	boost::mutex mMutexMap_;
	bool mbMapUpdated_;
};

} // end of namespace 

#endif // Map.h