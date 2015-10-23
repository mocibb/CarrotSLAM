#使用glog的静态库，添加预处理宏定义
if(MSVC)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D GLOG_NO_ABBREVIATED_SEVERITIES
/D GOOGLE_GLOG_DLL_DECL=")
endif(MSVC)

define_carrotslam_lib(
	# Lib name
	core		
	# Dependencies
	)


