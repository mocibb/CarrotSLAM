#include "core/carrot_slam.h"
#include <boost/algorithm/string.hpp>
#include <glog/logging.h>
#include <vector>
#include <algorithm>

using namespace std;

namespace carrotslam {

////SetSLAMEngineContext
SequenceSLAMEngine::SequenceSLAMEngine(const string& file, ISLAMEngineContextPtr& context){
  context_ = context;
  config_ = ISLAMEngineConfigPtr(new XMLSLAMEngineConfig(file));
}

struct NodeSeq {
  ISLAMNodePtr node;
  int seq;

  NodeSeq(ISLAMNodePtr& n, int s)
      : node(n),
        seq(s) {}

  bool operator<(const NodeSeq& rhs) const {
    return seq<rhs.seq;
  }
};

bool SequenceSLAMEngine::check() {
  for (auto n : nodes_) {
    if (!n->check())
      return false;
  }
  //reorder
  if (!is_ordered_) {
    int seq;
    vector<NodeSeq> nseq;
    nseq.reserve(nodes().size());
    for (auto n : nodes()) {
      seq = config_->getSeq(n->name());
      nseq.push_back(NodeSeq(n, seq));
    }

    sort(nseq.begin(), nseq.end());
    nodes_.clear();
    for (auto n : nseq) {
      nodes_.push_back(n.node);
    }

    is_ordered_ = true;
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

std::vector<ISLAMNodePtr> SequenceSLAMEngine::nodes(){
  return nodes_;
}

bool SequenceSLAMEngine::addNode(const ISLAMNodePtr& node){
  vector<string> nodes = config_->nodes();
  for (string n : nodes) {
    if (n == node->name()) {
      nodes_.push_back(node);
      return true;
    }
  }
  LOG(ERROR) << "not valid node " << node->name() <<"!" << endl;

  return false;
}

bool SequenceSLAMEngine::removeNode(const ISLAMNodePtr& node){
  for (auto it=nodes_.begin(); it != nodes_.end(); it++) {
    if ((*it)->name() == node->name()) {
      nodes_.erase(it);
      return true;
    }
  }

  return false;
}

ISLAMEngineConfigPtr SequenceSLAMEngine::getConfig() {
  return config_;
}

void SequenceSLAMEngine::getData(const std::string& name, ISLAMDataPtr& data) {
  context_->getData(name, data);
}

void SequenceSLAMEngine::setData(const std::string& name, const ISLAMDataPtr& data) {
  context_->setData(name, data);
}

////SetSLAMEngineContext
SetSLAMEngineContext::SetSLAMEngineContext() {
}

void SetSLAMEngineContext::getData(const string& name, ISLAMDataPtr& data) {
  if (this->container_.count(name) > 0) {
    //data.reset(this->container_[name].get()); this code is very buggy!!!!
    data = this->container_[name];
  } else {
    data.reset();
  }
}

void SetSLAMEngineContext::setData(const string& name, const ISLAMDataPtr& data) {
  this->container_[name] = data;
}

////XMLSLAMEngineConfig
XMLSLAMEngineConfig::XMLSLAMEngineConfig(const std::string& xml_file) {
  pugi::xml_parse_result result = doc_.load_file(xml_file.c_str());
  if (result.status != pugi::status_ok) {
    throw std::runtime_error(xml_file + " wrong xml!");
  }
}

vector<string> XMLSLAMEngineConfig::nodes() {
  std::vector<std::string> ret;
  pugi::xml_object_range<pugi::xml_node_iterator> nodes = doc_.select_node(
      "//nodes").node().children();
  for (auto node : nodes) {
    ret.push_back(node.attribute("name").value());
  }
  return ret;
}

string XMLSLAMEngineConfig::getValue(const std::string& nodeName,
                                     const string& attr) {
  std::string xpath = "/engine/nodes/node[@name='" + nodeName + "']";
  pugi::xpath_node nodes = doc_.select_node(xpath.c_str());
  pugi::xml_node node = nodes.node();
  pugi::xml_node nd = node.child(attr.c_str());

  if (nd == 0) {
    return "";
  }
  std::string value = nd.child_value();
  return value;
}

int XMLSLAMEngineConfig::getSeq(const std::string& nodeName) {
  std::string xpath = "/engine/nodes/node[@name='" + nodeName + "']";
  pugi::xpath_node nodes = doc_.select_node(xpath.c_str());
  pugi::xml_node node = nodes.node();
  return atoi(node.attribute("seq").value());
}

} // namespace carrotslam

