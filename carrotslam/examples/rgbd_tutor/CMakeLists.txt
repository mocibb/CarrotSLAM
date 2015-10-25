ADD_DEFINITIONS( -DTHIS SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

SET( LINKLIBS 
    carrotslam_core
    carrotslam_nodes
    carrotslam_types
    pugixml
    glog
)

ADD_EXECUTABLE( carrotslam_rgbdvo visualOdometry.cpp )
TARGET_LINK_LIBRARIES( carrotslam_rgbdvo
    ${LINKLIBS}
    ${OpenCV_LIBS}
    )



# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(carrotslam_rgbdvo PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(carrotslam_rgbdvo  PROPERTIES PROJECT_LABEL "(app) carrotslam_orbslam")
endif(ENABLE_SOLUTION_FOLDERS)
