/*************************************************************************
	> File Name: carrotslam/nodes/rgbd_tutorial_vo.cpp
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月13日 星期二 10时53分23秒
 ************************************************************************/

#include "nodes/rgbd_tutorial_vo.h"
#include "types/frame.h"
#include "types/feature.h"

#include <boost/lexical_cast.hpp>
using namespace carrotslam;

RGBDTutorial_VO :: RGBDTutorial_VO( const ISLAMEnginePtr& engine )
    //: engine_( engine )
{
    // 读取xml中的参数
    params_.detector_name   =   getValue<std::string> ("detector_name");
    params_.descriptor_name =   getValue<std::string> ("descriptor_name");
    params_.min_good_match  =   boost::lexical_cast<int> ( getValue<std::string> ("min_good_match") );
    params_.min_inliers     =   boost::lexical_cast<int> ( getValue<std::string> ("min_inliers") );
    params_.good_match_threshold =  boost::lexical_cast<double> ( getValue<std::string> ("good_match_threshold") );

    status_ = FIRST_FRAME;

    // 构建特征提取器
    cv::initModule_nonfree();
    detector_ = cv::FeatureDetector::create( params_.detector_name.c_str() );
    descriptor_ = cv::DescriptorExtractor::create( params_.descriptor_name.c_str() );
    last_pose_ = 0 ; 
}


std::shared_ptr<Frame> RGBDTutorial_VO::extractFeatures()
{
    std::shared_ptr<Frame> f ( new Frame() ); 
    std::vector< cv::KeyPoint > kp; 
    detector_ -> detect( last_pose_->color_image(), kp );
    cv::Mat desp;
    descriptor_ -> compute( last_pose_->color_image(), kp, desp );

    // 提出出来的东西 转换至Frame
    for ( size_t i=0; i<kp.size(); i++ )
    {
        FeaturePtr feature ( new Feature( std::move( kp[i] ) ) );
        feature->descriptor = std::move( desp.row(i) );
        f->features.push_back( feature );
    }

    return f;
}

bool RGBDTutorial_VO::check()
{
    return true;
}

bool RGBDTutorial_VO::isStart()
{
    return true;
}

bool RGBDTutorial_VO::isEnd()
{
    return true;
}

ISLAMNode::RunResult RGBDTutorial_VO::run()
{
    // 算法：将本帧和last_pose_进行比较，并返回结果
    if ( status_ == FIRST_FRAME )
    {
        initialize(); 
        return RUN_SUCCESS; 
    }
    else if (status_ == RUNNING )
    {
        // 尝试将当前帧与上一帧进行匹配
        try
        {
            compute();
        }
        catch( ... )
        {
            
        }
    }

    return RUN_SUCCESS; 
}

void RGBDTutorial_VO::compute() 
{
    ISLAMDataPtr new_data;
    engine_->getData( "dimage", new_data );
    
    // 提取new_data的特征
}
