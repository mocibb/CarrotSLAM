#ifndef CARROT_SLAM_H_
#define CARROT_SLAM_H_

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <iostream>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <pugixml.hpp>

namespace carrotslam {
class ISLAMData;
class ISLAMNode;
class ISLAMEngine;
class ISLAMEngineConfig;
class ISLAMEngineContext;

typedef std::shared_ptr<ISLAMData> ISLAMDataPtr;
typedef std::shared_ptr<ISLAMNode> ISLAMNodePtr;
typedef std::shared_ptr<ISLAMEngine> ISLAMEnginePtr;
typedef std::shared_ptr<ISLAMEngineConfig> ISLAMEngineConfigPtr;
typedef std::shared_ptr<ISLAMEngineContext> ISLAMEngineContextPtr;

/*! \brief base class for CarrotSLAM DATA.
 *
 */
class ISLAMData {
 public:
  ISLAMData(long id)
      : id_(id) {
  }
  virtual ~ISLAMData() {
    //DLOG(INFO) << "deconstructor of ISLAMData is called." << std::endl;
  }
  /*! read from disk */
  virtual bool read(std::istream& is) {
    return false;
  }
  /*! write out into disk */
  virtual bool write(std::ostream& os) {
    return false;
  }
  /*! each ISLAMData with different id */
  long id() {
    return id_;
  }
 protected:
  long id_;                //!< keep object id
};

/*! \brief base class for transmit data between nodes.
 *
 */
class ISLAMEngineContext {

 public:
  virtual ~ISLAMEngineContext() {
  }

  /*! 获取指定名字的数据 */
  template<typename T>
  void getData(const std::string& name, std::shared_ptr<T>& data){}

  /*! 保存指定名字的数据*/
  template<typename T>
  void setData(const std::string& name, const std::shared_ptr<T>& data){}
};

/*! \brief base class for SLAM pipeline.
 *
 */
class ISLAMEngine {
 public:
  ISLAMEngine(ISLAMEngineContextPtr& context)
      : context_(context) {
  }
  virtual ~ISLAMEngine() {
    //DLOG(INFO) << "deconstructor of ISLAMEngine is called." << std::endl;
  }
  /*! 在run之前检查所有Node需要满足的运行条件 */
  virtual bool check() = 0;
  /*! 运行所有的Node算法 */
  virtual void run() = 0;

  /*! 获取指定名字的数据 */
  template<typename T>
  void getData(const std::string& name, std::shared_ptr<T>& data) {
    context_->getData(name, data);
  }

  /*! 保存指定名字的数据*/
  template<typename T>
  void setData(const std::string& name, std::shared_ptr<T>& data) {
    context_->setData(name, data);
  }

 protected:
  ISLAMEngineConfigPtr config_;
  ISLAMEngineContextPtr context_;
  friend class ISLAMNode;
};

/*! \brief use to load parameter for each nodes.
 *
 */
class ISLAMEngineConfig {
 public:
  virtual ~ISLAMEngineConfig() {
  }
  template<typename T>
  T getValue(const std::string& xpath);

  template<typename T>
  T getValue(const std::string& xpath, T val);
};

/*! \brief base class for each algorithm runs.
 *
 */
class ISLAMNode {
 public:
  enum RunResult {
    RUN_FAILED,
    RUN_SUCCESS,
    RUN_NEXTROUND,
    RUN_FINISH
  };

  virtual ~ISLAMNode() {
  }

  /*! 检查Node需要满足的运行条件 */
  virtual bool check() = 0;
  /*! 是否为最初Node*/
  virtual bool isStart() = 0;
  /*! 是否为最终Node*/
  virtual bool isEnd() = 0;
  /*! 运行Node算法 */
  virtual RunResult run() = 0;

  /*! Node名字 在XML配置中使用 */
  virtual std::string name() {
    return name_;
  }

  /*! 获取Nodes配置参数*/
  template<typename T>
  T getValue(const std::string& name) {
    return engine_->config_->getValue<T>(name);
  }

  /*! 获取Nodes配置参数*/
  template<typename T>
  T getValue(const std::string& name, T val) {
    return engine_->config_->getValue<T>(name, val);
  }

 protected:
  ISLAMEnginePtr engine_;
  std::string name_;
};

/*! \brief 简单的ISLAMEngineContext的实现，所有的数据都放到内存中。
 *         因为SLAM中数据结构很大，所以需要一个更复杂的ISLAMEngineContext的实现
 */
class SetSLAMEngineContext : public ISLAMEngineContext {
 public:
  SetSLAMEngineContext();
  ~SetSLAMEngineContext() {
    //DLOG(INFO) << "deconstructor of SetSLAMEngineContext is called." << std::endl;
  }
  /*! 获取指定名字的数据 */
  template<typename T>
  void getData(const std::string& name, std::shared_ptr<T>& data) {
    if (this->container_.count(name) > 0) {
      //data.reset(this->container_[name].get()); this code is very buggy!!!!
      data = this->container_[name];
    } else {
      data.reset();
    }
  }

  /*! 保存指定名字的数据*/
  template<typename T>
  void setData(const std::string& name, const std::shared_ptr<T>& data) {
    this->container_[name] = data;
  }
 protected:
  std::map<std::string, ISLAMDataPtr> container_;
};

class XMLSLAMEngineConfig : public ISLAMEngineConfig {
 public:
  XMLSLAMEngineConfig(const std::string& xml_file) {
    pugi::xml_parse_result result = doc_.load_file(xml_file.c_str());
    if (result.status != pugi::status_ok) {
      throw std::runtime_error(xml_file + " wrong xml!");
    }
    node_ = doc_;
  }

  void chroot(const ISLAMNodePtr& node) {
    if (node.get() != nullptr) {
      std::string xpath = "//node[@name='" + node->name() + "']";
      node_ = doc_.select_node(xpath.c_str()).node();
    } else{
      node_ = doc_;
    }
  }

  std::vector<std::string> nodes() {
    std::vector<std::string> ret;
    pugi::xml_object_range<pugi::xml_node_iterator> nodes = doc_.select_node("//nodes").node().children();
    for (auto node : nodes ) {
      ret.push_back(node.attribute("name").value());
    }
    return ret;
  }

  template<typename T>
  T getValue(const std::string& name) {
    std::string value = node_.child(name.c_str()).child_value();
    if (std::is_same<T, int>::value) {
      return std::atoi(value.c_str());
    } else if (std::is_same<T, float>::value) {
      return std::atof(value.c_str());
    } else if (std::is_same<T, double>::value) {
      return std::atof(value.c_str());
    } else if (std::is_same<T, bool>::value){
      return (value == "true" ? true : false);
    } else if (std::is_same<T, std::string>::value){
      return value;
    }
  }

  template<typename T>
  T getValue(const std::string& name, T val) {
    pugi::xml_node nd = node_.child(name.c_str());
    if (nd == 0) {
      return val;
    }
    std::string value = nd.child_value();
    if (std::is_same<T, int>::value) {
      return std::atoi(value.c_str());
    } else if (std::is_same<T, float>::value) {
      return std::atof(value.c_str());
    } else if (std::is_same<T, double>::value) {
      return std::atof(value.c_str());
    } else if (std::is_same<T, bool>::value){
      return (value == "true" ? true : false);
    } else if (std::is_same<T, std::string>::value){
      return value;
    }
  }

 protected:
  std::string root_;
  pugi::xml_document doc_;
  pugi::xml_node node_;
};

/*! \brief 顺序执行的SLAMEngine
 *         所有Node都按顺序执行，如果Node执行失败，从第二个开始。
 *         第二个Node是初始化/重新初始化
 */
class SequenceSLAMEngine : public ISLAMEngine {
 public:
  SequenceSLAMEngine(const std::string& file, ISLAMEngineContextPtr& context)
      : ISLAMEngine(context) {

  }
  /*! 在run之前检查所有Node需要满足的运行条件 */
  bool check();
  /*! 运行所有的Node算法 */
  void run();

 private:
  std::vector<ISLAMNodePtr> nodes_;
};


} // namespace carrotslam
#endif /* CARROT_SLAM_H_ */
