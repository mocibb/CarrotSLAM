#include "core/carrot_slam.h"
#include <vector>

using namespace std;

namespace carrotslam {
bool SequenceSLAMEngine::check() {
  for (auto n : nodes_) {
    if (!n->check())
      return false;
  }
  return true;
}

void SequenceSLAMEngine::run() {
  int next = 0;
  while (1) {
    ISLAMNodePtr node = nodes_[next];
    //增加benchmark
    ISLAMNode::RunResult res = node->run();

    if (res == ISLAMNode::RUN_SUCCESS) {
      next++;
    } else if (res == ISLAMNode::RUN_FINISH) {
      break;
    } else {  //失败重新开始
      next = 0;
    }
  }
}

SetSLAMEngineContext::SetSLAMEngineContext() {
}

void SetSLAMEngineContext::getData(const std::string& name,
                                   ISLAMDataPtr& data) {
  if (this->container_.count(name) > 0) {
    //data.reset(this->container_[name].get()); this code is very buggy!!!!
    data = this->container_[name];
  } else {
    data.reset();
  }
}

void SetSLAMEngineContext::setData(const std::string& name,
                                   const ISLAMDataPtr& data) {
  this->container_[name] = data;
}
}

