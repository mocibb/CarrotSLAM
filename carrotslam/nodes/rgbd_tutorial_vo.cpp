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

#include <glog/logging.h>
using namespace carrotslam;

RGBDTutorial_VO :: RGBDTutorial_VO( const ISLAMEnginePtr& engine, const std::string& name )
    : ISLAMNode(  engine, name )
{
    // 读取xml中的参数
//    params_.detector_name   =   getValue<std::string> ("detector_name");
//    params_.descriptor_name =   getValue<std::string> ("descriptor_name");
//    params_.min_good_match  =   boost::lexical_cast<int> ( getValue<std::string> ("min_good_match") );
//    params_.min_inliers     =   boost::lexical_cast<int> ( getValue<std::string> ("min_inliers") );
//    params_.good_match_threshold =  boost::lexical_cast<double> ( getValue<std::string> ("good_match_threshold") );

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
        ComputeStatus status = compute();
    }

    return RUN_SUCCESS; 
}

RGBDTutorial_VO::ComputeStatus RGBDTutorial_VO::compute() 
{
    ISLAMDataPtr new_data; //新帧的RGBD数据
    engine_->getData( "dimage", new_data );
    std::shared_ptr<Frame> new_frame = extractFeatures( std::dynamic_pointer_cast<DImage> (new_data) ); //新一帧的Frame
    
    // 匹配特征
    std::vector<cv::DMatch> matches = match( this_frame_, new_frame );

    if ( matches.size() < this->params_.min_good_match )
    {
        LOG(WARNING) << "Good match is too few. Aborting this frame";
        return TOO_FEW_FEATURES;
    }

    // 计算ransac icp

    return OK;

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

    //筛选
    std::vector< cv::DMatch > goodMatches; 
    auto min_ele = std::min_element( matches.begin(), matches.end(), 
            [] (const cv::DMatch& m1, const cv::DMatch& m2 )
            { return m1.distance<m2.distance; });
    auto isGoodMatch = [min_ele, this] (const cv::DMatch& m) 
        {return m.distance < min_ele->distance * this->params_.good_match_threshold; } ; 
    auto iter = matches.begin();
    while ( true )
    {
        iter = find_if( iter, matches.end(), isGoodMatch );
        if ( iter != matches.end() )
            break;
        goodMatches.push_back( *iter );
        iter++;
    }

    LOG(INFO)<<"good matches: "<<goodMatches.size()<<std::endl;
    
    return std::move( goodMatches );
}

int RGBDTutorial_VO::solveRgbdPnP(
    std::shared_ptr<Frame> frame1, std::shared_ptr<DImage> image1, 
    std::shared_ptr<Frame> frame2, std::shared_ptr<DImage> image2, 
    const std::vector<cv::DMatch>& matches )
{
    std::vector< cv::Point3f > pts_obj; //三维点
    std::vector< cv::Point2f > pts_img; //二维点

    for ( auto m:matches )
    {
        Eigen::Vector2f& p = frame1->features[ m.queryIdx ]->px; 
    }
}
