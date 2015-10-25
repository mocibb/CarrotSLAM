<<<<<<< HEAD
#INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
ADD_LIBRARY(carrotslam_nodes
  tum_dataset_reader.cpp
  rgbd_tutorial_vo.cpp
  # orbslam_feature_extracting.cpp
  # feature_viewer.cpp
  # ORBextractor.cc
  # LoopClosing.cc
)

#ä½¿ç”¨glogçš„é™æ€åº“ï¼Œæ·»åŠ é¢„å¤„ç†å®å®šä¹‰
if(MSVC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D GLOG_NO_ABBREVIATED_SEVERITIES
/D GOOGLE_GLOG_DLL_DECL=")
endif(MSVC)
=======
#Ê¹ÓÃglogµÄ¾²Ì¬¿â£¬Ìí¼ÓÔ¤´¦Àíºê¶¨Òå
if(MSVC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D GLOG_NO_ABBREVIATED_SEVERITIES
/D GOOGLE_GLOG_DLL_DECL=")
endif(MSVC)

define_carrotslam_lib(
	# Lib name
	nodes		
	# Dependencies
	)
>>>>>>> 4d353c9c322d70228eadcfe5f968da26891756d4
