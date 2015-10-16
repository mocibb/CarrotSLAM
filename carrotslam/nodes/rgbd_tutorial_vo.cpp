/*************************************************************************
	> File Name: carrotslam/nodes/rgbd_tutorial_vo.cpp
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月13日 星期二 10时53分23秒
 ************************************************************************/

#include "nodes/rgbd_tutorial_vo.h"
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

    last_pose_ = 0 ; 
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
        // 初始化
        ISLAMDataPtr data; 
        engine_->getData( "dimage", data );
        last_pose_ = dynamic_cast< DImage* > data;
        
        
    }

    return RUN_SUCCESS; 
}
