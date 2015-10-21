/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   tum_dataset_reader.cpp
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

#include "nodes/tum_dataset_reader.h"
#include "types/dimage.h"
#include "core/common.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>


#include <glog/logging.h>
#include <boost/filesystem.hpp>

namespace carrotslam {
using namespace std;
using namespace google;
using namespace boost::algorithm;

TUMDatasetReader::TUMDatasetReader(const ISLAMEnginePtr& engine, const string& name)
    : cnt_(0),
      ISLAMNode(engine, name) {
  dataset_dir_ = getTypedValue<string>(this, "datasetDir");

  string rgb_txt_path_ = dataset_dir_ + "/rgb.txt";
  string depth_txt_path_ = dataset_dir_ + "/depth.txt";
  if (!boost::filesystem::exists(rgb_txt_path_)) {
    throw std::runtime_error("rgb.txt not exist!");
  }
  if (!boost::filesystem::exists(depth_txt_path_)) {
    throw std::runtime_error("depth.txt not exist!");
  }

  ifstream rgb_txt_istream_(rgb_txt_path_);
  ifstream depth_txt_istream_(depth_txt_path_);
  string rgb_line_, depth_line_;

  {
    //ScopeTime(LOG(INFO), "TUMDatasetReader reading dataset");
    //ScopeTime(std::cerr, "TUMDatasetReader reading dataset");
    while (getline(rgb_txt_istream_, rgb_line_)) {
      if (rgb_line_[0] != '#') {
        std::vector<std::string> strs;
        boost::split(strs, rgb_line_, is_space());
        rgb_dataset_.push_back(TUMDatasetImageLine(strs[0], strs[1]));
      }
    }

    while (getline(depth_txt_istream_, depth_line_)) {
      if (depth_line_[0] != '#') {
        std::vector<std::string> strs;
        boost::split(strs, depth_line_, is_space());
        depth_dataset_.push_back(TUMDatasetImageLine(strs[0], strs[1]));
      }
    }

    if (rgb_dataset_.size() != depth_dataset_.size()) {
      //LOG(ERROR) << "" << endl;
      LOG(WARNING)<<"size of rgb and depth is not matched" << endl;
    }
  }

}

ISLAMNode::RunResult TUMDatasetReader::run() {
  if (cnt_ < rgb_dataset_.size()) {
    ISLAMDataPtr dimage(
        new DImage(dataset_dir_ + "/" + rgb_dataset_[cnt_].filename,
                   dataset_dir_ + "/" + depth_dataset_[cnt_].filename));
    engine_->setData("dimage", dimage);
    cnt_++;
    return RUN_SUCCESS;
  }
  return RUN_FINISH;
}

bool TUMDatasetReader::check() {
  return true;
}

bool TUMDatasetReader::isStart() {
  return false;
}

bool TUMDatasetReader::isEnd() {
  return false;
}

}   // namespace carrotslam

