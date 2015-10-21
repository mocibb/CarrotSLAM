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


std::shared_ptr<Frame> RGBDTutorial_VO::extractFeatures( const std::shared_ptr<DImage> & image )
{
    std::shared_ptr<Frame> f ( new Frame() ); 
    std::vector< cv::KeyPoint > kp; 
    detector_ -> detect( image->color_image(), kp );
    cv::Mat desp;
    descriptor_ -> compute( image->color_image(), kp, desp );

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
    std::shared_ptr<Frame> new_frame = extractFeatures( new_data );
    
    // 匹配特征
    std::vector<cv::DMatch> matches = match( this_frame_, new_frame );

}

std::vector<cv::DMatch> && RGBDTutorial_VO::match( const std::shared_ptr<Frame>& p1, const std::shared_ptr<Frame>& p2 )
{
    static cv::FlannBasedMatcher matcher; 
    // 调用opencv::FlannBasedMatcher进行匹配
    cv::Mat feature1( p1->features.size(), p1->features[0]->descriptor.cols, CV_32F ); 
    cv::Mat feature2( p2->features.size(), p2->features[0]->descriptor.cols, CV_32F ); 

    for ( int i=0; i<p1->features.size(); i++ )
    {
        cv::Mat& d = p1->features[i]->descriptor;
        d.row(0).copyTo( feature1.row(i) );
    }
    
    for ( int i=0; i<p2->features.size(); i++ )
    {
        cv::Mat& d = p2->features[i]->descriptor;
        d.row(0).copyTo( feature2.row(i) );
    }

    std::vector<cv::DMatch> matches;
    matcher.match( feature1, feature2, matches );
    return matches;
}
