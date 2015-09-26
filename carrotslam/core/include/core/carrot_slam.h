#ifndef CARROT_SLAM_H_
#define CARROT_SLAM_H_

#include <string>
#include <vector>
#include <memory>
#include <set>

namespace carrotslam {
class ISLAMData;
class ISLAMNode;
class ISLAMEngine;
class ISLAMEngineContext;

typedef std::shared_ptr<ISLAMData> ISLAMDataPtr;
typedef std::shared_ptr<ISLAMNode> ISLAMNodePtr;
typedef std::shared_ptr<ISLAMEngine> ISLAMEnginePtr;
typedef std::shared_ptr<ISLAMEngineContext> ISLAMEngineContextPtr;

/*! \brief 封装SLAM中用到的数据结构.
 *
 */
class ISLAMData {
 public:
  virtual ~ISLAMData() {
  }

};

/*! \brief ISLAMEngineContext用于SLAM中数据流的传递。
 *
 */
class ISLAMEngineContext {

 public:
  virtual ~ISLAMEngineContext() {
  }

  /*! 获取指定名字的数据 */
  virtual void getData(const std::string& name, ISLAMDataPtr& data) = 0;

  /*! 保存指定名字的数据*/
  virtual void setData(const std::string& name, const ISLAMDataPtr& data) = 0;
};

/*! \brief ISLAMNode封装SLAM的算法单元.
 *
 */
class ISLAMNode {
 public:
  enum RunResult {
    RUN_FAILED,
    RUN_SUCCESS,
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

 protected:
  ISLAMEnginePtr engine_;
};

/*! \brief 封装SLAM中用到的数据结构.
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

  /*! 获取指定名字的数据 */
  virtual void getData(const std::string& name, ISLAMDataPtr& data) {
    context_->getData(name, data);
  }

  /*! 保存指定名字的数据*/
  virtual void setData(const std::string& name, const ISLAMDataPtr& data) {
    context_->setData(name, data);
  }

 protected:
  ISLAMEngineContextPtr context_;
};

/*! \brief 简单的ISLAMEngineContext的实现，所有的数据都放到内存中。
 *         因为SLAM中数据结构很大，所以需要一个更复杂的ISLAMEngineContext的实现
 */
class SetSLAMEngineContext : ISLAMEngineContext {
 public:
  /*! 获取指定名字的数据 */
  void getData(const std::string& name, ISLAMDataPtr& data);

  /*! 保存指定名字的数据*/
  void setData(const std::string& name, const ISLAMDataPtr& data);

 protected:
  std::set<std::string, ISLAMDataPtr> container_;
};

/*! \brief 顺序执行的SLAMEngine
 *         所有Node都按顺序执行，如果Node执行失败，从第二个开始。
 *         第二个Node是初始化/重新初始化
 */
class SequenceSLAMEngine : ISLAMEngine {
 public:
  SequenceSLAMEngine(const std::string& file, ISLAMEngineContext& context);
  /*! 在run之前检查所有Node需要满足的运行条件 */
  bool check();
  /*! 运行所有的Node算法 */
  void run();

 private:
  std::vector<ISLAMNodePtr> nodes_;
};
} // namespace carrotslam
#endif /* CARROT_SLAM_H_ */
