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
  ISLAMEngineContextPtr context_(new SetSLAMEngineContext());
  ISLAMEnginePtr engine_(new SequenceSLAMEngine("", context_));
  TUMDatasetReader reader(engine_, "/home/mocibb/dataset/tum/rgbd_dataset_freiburg1_floor");
  reader.run();

  ISLAMDataPtr data;
  context_->getData("dimage2", data);
  if (data.get() != nullptr)
    throw std::runtime_error("should be nullptr");

  context_->getData("dimage", data);

  DImage* dimage = static_cast<DImage*>(data.get());
  imshow("test", dimage->color_image());
  waitKey(0);

  return 0;

}
