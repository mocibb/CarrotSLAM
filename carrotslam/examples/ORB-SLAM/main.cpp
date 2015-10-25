#include "core/carrot_slam.h"
#include "nodes/tum_dataset_reader.h"
#include "nodes/orbslam_feature_extracting.h"
#include "nodes/feature_viewer.h"
#include "types/dimage.h"
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
  ISLAMEnginePtr engine(new SequenceSLAMEngine(std::string(THIS_SOURCE_DIR) +"/orbslam.xml", context));
  engine->addNode(ISLAMNodePtr(new TUMDatasetReader(engine, "tum_dataset_reader")));
  engine->addNode(ISLAMNodePtr(new ORBSLAMFeatureExtracting(engine, "orbslam_feature_extracting")));
  engine->addNode(ISLAMNodePtr(new FeatureViewer(engine, "feature_viewer")));


  engine->check();
  engine->run();

  return 0;

}
