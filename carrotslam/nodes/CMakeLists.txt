#INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
ADD_LIBRARY(carrotslam_nodes
  tum_dataset_reader.cpp
  rgbd_tutorial_vo.cpp
  orbslam_feature_extracting.cpp
  feature_viewer.cpp
  ORBextractor.cpp
  # LoopClosing.cc
)

#使用glog的静态库，添加预处理宏定义
if(MSVC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D GLOG_NO_ABBREVIATED_SEVERITIES
/D GOOGLE_GLOG_DLL_DECL=")
endif(MSVC)
