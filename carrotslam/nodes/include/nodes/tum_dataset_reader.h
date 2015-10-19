/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   tum_dataset_reader.h
 * Date:   2015.09.30
 * Func:   tum dataset reader
 *
 *
 * The MIT License (MIT)
 * Copyright (c) 2015 CarrotSLAM Group
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef NODES_TUM_DATASET_READER_H_
#define NODES_TUM_DATASET_READER_H_

#include <core/carrot_slam.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace carrotslam {
/*! \brief image object that keep id
 *
 *
 *  Internally use cv::Mat of opencv for holding image object
 */
class TUMDatasetReader : public ISLAMNode {
 private:
  struct TUMDatasetImageLine {
    TUMDatasetImageLine(const std::string& ts, const std::string& fn)
        : timestamp(ts),
          filename(fn) {
    }
    std::string timestamp;
    std::string filename;
  };
 public:
  /*!
   the constructor of image
   @param img_path the path to image file
   */
  TUMDatasetReader(const ISLAMEnginePtr& engine, const std::string& name);

  ~TUMDatasetReader(){
  };

  RunResult run();

  inline bool check();

  bool isStart();

  bool isEnd();

 protected:
  std::vector<TUMDatasetImageLine> rgb_dataset_;
  std::vector<TUMDatasetImageLine> depth_dataset_;
  std::string dataset_dir_;
  long cnt_;

};
}   // namespace carrotslam

#endif /* NODES_TUM_DATASET_READER_H_ */
