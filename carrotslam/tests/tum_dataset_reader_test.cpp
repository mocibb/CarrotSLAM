#include "nodes/tum_dataset_reader.h"
#include "types/dimage.h"
#include "core/carrot_slam.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace carrotslam;
using namespace cv;
long carrotslam::DImage::image_id_ = 0;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    return -1;
  }
  ISLAMEngineContextPtr context_(new SetSLAMEngineContext());
  //"../carrotslam/examples/ORB-SLAM/orbslam.xml"
  ISLAMEnginePtr engine_(new SequenceSLAMEngine(argv[1], context_));

  ISLAMNodePtr reader(new TUMDatasetReader(engine_, "tum_dataset_reader"));
  engine_->addNode(reader);
  reader->run();

  std::shared_ptr<DImage> dimage = getSLAMData<DImage>(engine_, "dimage");
  imshow("test", dimage->color_image());
  waitKey(0);

  return 0;

}
