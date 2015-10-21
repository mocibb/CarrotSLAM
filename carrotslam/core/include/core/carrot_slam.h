#ifndef CARROT_SLAM_H_
#define CARROT_SLAM_H_

#include <string>
#include <cstdlib>
#include <vector>
#include <memory>
#include <map>
#include <iostream>
#include <boost/lexical_cast.hpp>
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
  ISLAMData(  ) : id_(0) {}
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
  virtual void getData(const std::string& name, ISLAMDataPtr& data) = 0;

  /*! 保存指定名字的数据 */
  virtual void setData(const std::string& name, const ISLAMDataPtr& data) = 0;

};

/*! \brief base class for SLAM pipeline.
 *
 */
class ISLAMEngine {

 public:
  virtual ~ISLAMEngine() {
  }

  /*! 在run之前检查所有Node需要满足的运行条件 */
  virtual bool check() = 0;
  /*! 运行所有的Node算法 */
  virtual void run() = 0;
  /*! 获取所有的Node */
  virtual std::vector<ISLAMNodePtr> nodes() = 0;
  /*! 追加运行Node */
  virtual bool addNode(const ISLAMNodePtr& node) = 0;
  /*! 删除运行Node */
  virtual bool removeNode(const ISLAMNodePtr& node) = 0;
  /*! 获取指定名字的数据 */
  virtual void getData(const std::string& name, ISLAMDataPtr& data) = 0;
  /*! 保存指定名字的数据*/
  virtual void setData(const std::string& name, const ISLAMDataPtr& data) = 0;
  /*! 获取参数配置信息 */
  virtual ISLAMEngineConfigPtr getConfig() = 0;

};

/*! \brief use to load parameter for each nodes.
 *
 */
class ISLAMEngineConfig {
 public:
  virtual ~ISLAMEngineConfig() {
  }

  /*! 获取所有的Node的名称 */
  virtual std::vector<std::string> nodes() = 0;
  /*! 获取的Node的配置参数 */
  virtual std::string getValue(const std::string& nodeName, const std::string& xpath) = 0;
  /*! 获取的Node的顺序 */
  virtual int getSeq(const std::string& nodeName) = 0;
};

class XMLSLAMEngineConfig : public ISLAMEngineConfig {
 public:
  XMLSLAMEngineConfig(const std::string& xml_file);

  virtual ~XMLSLAMEngineConfig() {
  }

  std::vector<std::string> nodes();

  std::string getValue(const std::string& nodeName, const std::string& xpath);

  int getSeq(const std::string& nodeName);

 protected:
  pugi::xml_document doc_;
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

  ISLAMNode(const ISLAMEnginePtr& engine, const std::string& name)
      : engine_(engine),
        name_(name) {
  }


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
  std::string getValue(const std::string& attr) {
    ISLAMEngineConfigPtr config_ = engine_->getConfig();
    //FIXME: 如何能写成通用形式
    return config_->getValue(name(), attr);
  }

 protected:
  ISLAMEnginePtr engine_;
  std::string name_;
};

/*!
 * 获取node下的指定参数值，如果指定参数不存在抛出异常
 */
template<typename T>
T getTypedValue(ISLAMNode* node, const std::string& attr) {
  std::string str_val = node->getValue(attr);

  if (str_val == "")
    throw std::runtime_error("attribute " + attr + " not found!");

  return boost::lexical_cast<T>(str_val);
}

/*!
 * 获取node下的指定参数值，如果指定参数不存在用默认的代替
 */
template<typename T>
T getTypedValue(ISLAMNode* node, const std::string& attr, T val) {
  std::string str_val = node->getValue(attr);

  if (str_val == "")
    return val;

  return boost::lexical_cast<T>(str_val);
}

/*!
 * 获取指定类型的ISLAMData数据
 */
template<typename T>
std::shared_ptr<T>
getSLAMData(const ISLAMEnginePtr& engine, const std::string& name) {
  ISLAMDataPtr data;
  engine->getData(name, data);
  return std::dynamic_pointer_cast<T>(data);
}

/*!
 * 设置指定类型的ISLAMData数据
 */
template<typename T>
void setSLAMData(const ISLAMEnginePtr& engine, const std::string& name,
                 std::shared_ptr<T>& data) {
  engine->setData(name, std::dynamic_pointer_cast<ISLAMData>(data));
}

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
  void getData(const std::string& name, ISLAMDataPtr& data);

  /*! 保存指定名字的数据*/
  void setData(const std::string& name, const ISLAMDataPtr& data);

 protected:
  std::map<std::string, ISLAMDataPtr> container_;
};



/*! \brief 顺序执行的SLAMEngine
 *         所有Node都按顺序执行，如果Node执行失败，从第二个开始。
 *         第二个Node是初始化/重新初始化
 */
class SequenceSLAMEngine : public ISLAMEngine {
 public:
  SequenceSLAMEngine(const std::string& file, ISLAMEngineContextPtr& context);

  ~SequenceSLAMEngine() {
  }
  /*! 在run之前检查所有Node需要满足的运行条件 */
  bool check();
  /*! 运行所有的Node算法 */
  void run();

  std::vector<ISLAMNodePtr> nodes();

  bool addNode(const ISLAMNodePtr& node);

  bool removeNode(const ISLAMNodePtr& node);

  void getData(const std::string& name, ISLAMDataPtr& data);

  void setData(const std::string& name, const ISLAMDataPtr& data);


 protected:
  ISLAMEngineConfigPtr getConfig();

  ISLAMEngineConfigPtr config_;
  ISLAMEngineContextPtr context_;
  std::vector<ISLAMNodePtr> nodes_;
  bool is_ordered_ = false;
};


} // namespace carrotslam
#endif /* CARROT_SLAM_H_ */
