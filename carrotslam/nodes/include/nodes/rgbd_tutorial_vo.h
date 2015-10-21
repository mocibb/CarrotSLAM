/*************************************************************************
	> File Name: include/nodes/rgbd_tutorial_vo.h
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月11日 星期日 16时53分36秒

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
 ************************************************************************/

#ifndef RGBD_TUTORIAL_VO_H
#define RGBD_TUTORIAL_VO_H


// std c++
#include <iostream>
#include <fstream>
#include <vector>

// Eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// pcl
// 然而暂时用不到pcl

// carrotslam
#include "core/carrot_slam.h"
#include "types/dimage.h"
#include "types/frame.h"

using namespace carrotslam;
namespace carrotslam {

/** \class RGBDTutorial_VO
 * \brief 一起做系列的VO类
 * 读取RGB-D图像 DImage
 * 输出该图像对应的帧：Frame
 */

/** \brief RGBD Tutorial VO 的参数配置结构体
 * 列举了常用的几个参数，都从xml里读取
 */
struct RGBDTUTORIALVO_PARAMS
{
    std::string detector_name;   
    std::string descriptor_name;
    int min_good_match;   //最小匹配数量
    int min_inliers;      //ransac pnp的最小inliers
    double good_match_threshold;   //筛选goodmatch的倍数
};

class RGBDTutorial_VO : public ISLAMNode
{
public:
    /* 自身的状态枚举：第一帧（初始化），正常运行，丢失
     */
    enum VOStatus {
        FIRST_FRAME, RUNNING, LOST
    }; 
    /**
     * constructor
     */
    RGBDTutorial_VO( const ISLAMEnginePtr& engine, const std::string& name );

    ~RGBDTutorial_VO(); 

    // 继承函数
    virtual RunResult run();
    
    bool check();

    bool isStart();

    bool isEnd();

    // 自有函数
protected:

    typedef enum  { 
        OK=0, TOO_FEW_FEATURES, 
    }ComputeStatus;


    /**
     * \brief 读取第一帧的数据
     */
    void initialize() 
    {
        ISLAMDataPtr data; 
        engine_->getData( "dimage", data  );
        last_pose_ = std::dynamic_pointer_cast< DImage > (data);
        this_frame_ = extractFeatures( last_pose_ ); 
        status_ = RUNNING;
    }

    // \brief 计算与last_pose_的相对位姿并输出结果
    RGBDTutorial_VO::ComputeStatus compute(); 

    // \brief 提取last_pose里的特征，使用Frame接口
    std::shared_ptr<Frame> extractFeatures( const std::shared_ptr<DImage> & image ); 

    // \brief 匹配特征
    std::vector<cv::DMatch> && match( const std::shared_ptr<Frame>& p1, const std::shared_ptr<Frame>& p2 );

    // \brief 计算ransac pnp
    // \param 两帧各自的frame(含特征)和rgbd原图(计算特征位置)
    // \param 特征匹配关系
    int solveRgbdPnP( 
            std::shared_ptr<Frame> frame1, std::shared_ptr<DImage> image1, 
            std::shared_ptr<Frame> frame2, std::shared_ptr<DImage> image2, 
            const std::vector<cv::DMatch>& matches );
protected:
    // 数据成员
    std::shared_ptr<DImage>     last_pose_;     // 上一帧图像
    std::shared_ptr<Frame>      this_frame_;    //当前帧
    RGBDTUTORIALVO_PARAMS       params_;        //参数配置
    VOStatus                    status_;        //自身的状态

    // 特征相关
    cv::Ptr< cv::FeatureDetector > detector_; 
    cv::Ptr< cv::DescriptorExtractor > descriptor_; 

};


} // end of namespace

#endif
