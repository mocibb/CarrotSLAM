/*************************************************************************
	> File Name: examples/rgbd_tutor/visualOdometry.cpp
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月25日 星期日 16时46分17秒
 ************************************************************************/

/**
 * An example of setting a rgbd visual odometry for tum dataset
 */

#include "nodes/rgbd_tutorial_vo.h"

#include <iostream>

using namespace std;
using namespace carrotslam;

void usage() 
{
    cout<<"Usage: rgbdvo [xmlconfig=./examples/rgbd_tutor/visualodometry.xml]"<<endl;
}

int main( int argc, char** argv )
{

    ISLAMEngineContextPtr context_ ( new SetSLAMEngineContext() );    
    ISLAMEnginePtr engine_ ( new SequenceSLAMEngine( 
                "./examples/rgbd_tutor/visualodometry.xml", context ));

    engine->addNode( ISLAMNodePtr( new TUMDatasetReader(engine, "tum_dataset_reader") ) );
    engine->addNode( ISLAMNodePtr( new RGBDTutorial_VO(engine, "rgbd_tutorial_vo") ) );

    engine->check();
    engine->run();

    return 0;
}
