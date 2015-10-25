/*************************************************************************
	> File Name: carrotslam/nodes/rgbd_tutorial_vo.cpp
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月13日 星期二 10时53分23秒
 ************************************************************************/

#include "nodes/rgbd_tutorial_vo.h"
#include "types/frame.h"
#include "types/feature.h"
#include "core/common.h"

#include <boost/lexical_cast.hpp>

#include <glog/logging.h>
using namespace carrotslam;

RGBDTutorial_VO :: RGBDTutorial_VO( const ISLAMEnginePtr& engine, const std::string& name )
    : ISLAMNode(  engine, name )
{
    // 读取xml中的参数
    params_.detector_name   =   getValue ("detector_name");
    params_.descriptor_name =   getValue ("descriptor_name");
    params_.min_good_match  =   boost::lexical_cast<int> ( getValue ("min_good_match") );
    params_.min_inliers     =   boost::lexical_cast<int> ( getValue ("min_inliers") );
    params_.good_match_threshold =  boost::lexical_cast<double> ( getValue ("good_match_threshold") );
    params_.max_trans_frames    =  boost::lexical_cast<double> ( getValue ("max_trans_frames") );

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

    } else //status_ == LOST 
    {
        initialize();
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
        LOG(WARNING) << "Good match is too few. Aborting this frame" << std::endl;
        status_ = LOST;
        return TOO_FEW_FEATURES;
    }

    // 计算ransac icp
    Sophus::SE3f T;
    int result = solveRgbdPnP( this_frame_, last_pose_, 
            new_frame, std::dynamic_pointer_cast<DImage>( new_data ),
            matches, T );

    // check if the result is valid
    if ( result != 0 )
    {
        status_ = LOST;
        return PNP_FAILED;
    }

    //set the new frame 
    new_frame -> T_f_w = T * this_frame_ -> T_f_w;

    last_pose_ = std::dynamic_pointer_cast<DImage> ( new_data );
    this_frame_ = new_frame;

    ISLAMDataPtr data = std::dynamic_pointer_cast<ISLAMData>( this_frame_ );
    engine_->setData( "frame", this_frame_ );

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
    const std::vector<cv::DMatch>& matches, 
    Sophus::SE3f& T )
{
    std::vector< cv::Point3f > pts_obj; //三维点
    std::vector< cv::Point2f > pts_img; //二维点

    for ( auto m:matches )
    {
        Eigen::Vector2d p = frame1->features[ m.queryIdx ]->px.cast<double> (); 
        ushort d = image1->depth_image().ptr<ushort> ( int(p[1]) ) [ int(p[0]) ];
        if (d == 0)         //深度图中该像素无读数
            continue;
        // 将p和d转换至3D空间坐标
        Eigen::Vector3d point3d = frame1->cam->cam2world( p, d );
        pts_obj.push_back( cv::Point3f( point3d[0], point3d[1], point3d[2] ) );
        pts_img.push_back( cv::Point2f( frame2->features[ m.trainIdx ]->px[0] , frame2->features[ m.trainIdx ]->px[1]) );
    }
    if (pts_obj.size() ==0 || pts_img.size()==0 )
    {
        LOG(WARNING)<<"RGBDPNP::Not enough points"<<std::endl;
        return -1;
    }
   
    // 构建Camera矩阵
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts_obj, pts_img, frame1->cam->getCvMat(), cv::Mat(), 
            rvec, tvec, false, 100, 1.0 ,100, inliers );

    if ( inliers.rows < this->params_.min_inliers )
    {
        LOG(WARNING)<<"RGBDPNP::Not enough inliers"<<std::endl;
        return -1;
    }
    
    // check the norm
    double n = cv::norm( tvec ) + cv::norm( rvec );
    if ( n>params_.max_trans_frames )
    {
        LOG(WARNING) << "RGBDPNP::Too large transform, aborted." << std::endl;
        return -1;
    }
    
    T = cvMat2SophusSE3f( rvec, tvec );
    return 0;
}
