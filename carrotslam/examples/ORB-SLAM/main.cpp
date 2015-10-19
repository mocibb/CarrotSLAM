#include "nodes/tum_dataset_reader.h"
#include "nodes/tum_dataset_reader.h"
#include "types/dimage.h"
#include "core/carrot_slam.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace carrotslam;
using namespace cv;
long carrotslam::DImage::image_id_ = 0;
int main() {
  ISLAMEngineContextPtr context(new SetSLAMEngineContext());
  ISLAMEnginePtr engine(new SequenceSLAMEngine("/home/mocibb/works/CarrotSLAM/carrotslam/examples/ORB-SLAM/orbslam.xml", context));
 // vector<string> nodes = engine->nodeNames();
  //cout << nodes[0] << endl;
  //TUMDatasetReader reader(engine, "tum_dataset_reader");


  engine->check();
  engine->run();

  return 0;

}
